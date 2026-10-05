// =====================================================================
// mic_input.cpp - INMP441 (I2S-MEMS-Mikrofon) über PIO + DMA
// =====================================================================
// Ausgelagert aus main.cpp (Refactoring, siehe DEVLOG). Inhalt
// unverändert: PIO-I2S-Empfänger auf pio1, DMA in einen Ringpuffer,
// Füllstand-Steuerung, Umrechnung 24 Bit -> Q16 mit Gain, DC-Nachführung.

#include "mic_input.h"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "i2s_mic.pio.h"
#include <cmath>

namespace {

// =====================================================================
// INMP441-Eingang: PIO (I2S-Master) -> DMA -> Ringpuffer
// =====================================================================

// Pegelanpassung: Das INMP441 hat keinen einstellbaren Gain, normale
// Sprache landet grob bei -50 bis -60 dBFS. Umrechnung 24 Bit -> Q16:
//   q16 = (s24 << kMicGainShift) >> 7
// kMicGainShift = 0 -> Vollausschlag des Mics = 1.0 in Q16
// jede +1 = +6 dB. Startwert 6 (+36 dB): Sprache landet damit grob in
// der Größenordnung, die der alte ADC-Pfad lieferte (Vollausschlag dort
// ebenfalls 1.0). Über micMin/micMax in der Diagnose nachkalibrieren -
// Ziel: normale Sprache mit Spitzen um ~200-500/1000. Max. 7.
constexpr int kMicGainShift = 3; // war 6 - Spitzen lagen bei ±4000-5600/1000, siehe DEVLOG

// Ringpuffer: 1024 Wörter = 4096 Bytes. Der DMA-Ring-Modus verlangt,
// dass der Puffer auf seine eigene Größe ausgerichtet ist.
constexpr uint32_t kMicRingWords    = 1024;
constexpr uint32_t kMicRingMask     = kMicRingWords - 1;
constexpr uint     kMicRingSizeBits = 12; // 2^12 Bytes = 4096
uint32_t g_micRing[kMicRingWords] __attribute__((aligned(kMicRingWords * sizeof(uint32_t))));

// Füllstand-Steuerung: Mic (PIO-Takt) und DAC (pico-extras-PIO-Takt)
// laufen beide vom Systemtakt ab, aber mit unterschiedlich gerundeten
// Teilern - sie driften minimal gegeneinander (grob ein Sample pro
// Sekunde oder weniger). Wir halten einen Vorlauf von kMicTargetFill
// Samples als Puffer gegen Schwankungen der Schleifenlaufzeit (z.B.
// printf alle 100 Puffer). Kosten: kMicTargetFill / kSampleRateHz
// zusätzliche Latenz (512 -> ~23 ms). Kann später verkleinert werden,
// falls micUnder/micOver in der Diagnose dauerhaft 0 bleiben.
constexpr uint32_t kMicMaxFill    = 896; // darüber: Lesezeiger nachziehen (deutlich unter Ringgröße!)

PIO      g_micPio = pio1;  // pio0 belegt pico-extras für den DAC
uint     g_micSm = 0;
int      g_micDmaChannel = -1;
uint32_t g_micReadIdx = 0;
uint32_t g_micUnderruns = 0;
uint32_t g_micOverruns = 0;

// Gleichspannungs-Nachführung bleibt erhalten (siehe DEVLOG Nachtrag
// 17) - das INMP441 hat zwar einen internen Hochpass, aber gerade in
// den ersten Sekunden nach dem Start einen deutlichen, abklingenden
// Offset (im hello_mic_test sichtbar). Kostet fast nichts.
constexpr float kMicDcTrackingMs = 300.0f;
q16 g_micDcState = 0;
q16 g_micDcUpdateRate = 0;

} // namespace

void setup_mic() {
    g_micDcUpdateRate = kQ16One - float_to_q16(expf(-1.0f / (0.001f * kMicDcTrackingMs * (float)kSampleRateHz)));

    g_micSm = (uint)pio_claim_unused_sm(g_micPio, true);
    uint offset = pio_add_program(g_micPio, &i2s_mic_in_program);
    pio_sm_config c = i2s_mic_in_program_get_default_config(offset);

    // SCK (Basis) + WS (Basis+1) als side-set-Ausgänge
    sm_config_set_sideset_pins(&c, kMicSckPin);
    pio_gpio_init(g_micPio, kMicSckPin);
    pio_gpio_init(g_micPio, kMicWsPin);
    pio_sm_set_consecutive_pindirs(g_micPio, g_micSm, kMicSckPin, 2, true);
    // Weichere Flanken auf den Taktleitungen - weniger Einstreuung in
    // die restliche Schaltung, siehe DEVLOG (Übersprechen).
    gpio_set_slew_rate(kMicSckPin, GPIO_SLEW_RATE_SLOW);
    gpio_set_slew_rate(kMicWsPin, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(kMicSckPin, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(kMicWsPin, GPIO_DRIVE_STRENGTH_2MA);

    // SD als Eingang, mit Pulldown (INMP441 schaltet SD außerhalb seines
    // Slots und beim I2S-Verzögerungsbit hochohmig).
    sm_config_set_in_pins(&c, kMicSdPin);
    pio_gpio_init(g_micPio, kMicSdPin);
    gpio_pull_down(kMicSdPin);
    pio_sm_set_consecutive_pindirs(g_micPio, g_micSm, kMicSdPin, 1, false);

    // MSB zuerst, Autopush nach 32 Bit, RX-FIFO verdoppelt (TX unbenutzt).
    sm_config_set_in_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

    // Exakt 128 PIO-Takte pro Frame -> Samplerate = sys_clk / (128 * div).
    // Über die Config setzen - pio_sm_init() übernimmt den Teiler aus c
    // (ein vorheriges pio_sm_set_clkdiv() würde überschrieben).
    float div = (float)clock_get_hz(clk_sys) / ((float)kSampleRateHz * 128.0f);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(g_micPio, g_micSm, offset, &c);

    // DMA: RX-FIFO -> Ringpuffer, getaktet über den DREQ der State
    // Machine (ein Transfer pro Sample). Ring auf der Schreibseite: die
    // Schreibadresse läuft nach 4096 Bytes automatisch zum Pufferanfang
    // zurück. Transferzähler maximal - reicht bei 22,05 kHz für ~54 h,
    // mic_read_block() startet den Kanal danach einfach neu.
    g_micDmaChannel = dma_claim_unused_channel(true);
    dma_channel_config dc = dma_channel_get_default_config((uint)g_micDmaChannel);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_ring(&dc, true, kMicRingSizeBits);
    channel_config_set_dreq(&dc, pio_get_dreq(g_micPio, g_micSm, false));
    dma_channel_configure((uint)g_micDmaChannel, &dc,
                          g_micRing, &g_micPio->rxf[g_micSm],
                          0xFFFFFFFFu, true);

    pio_sm_set_enabled(g_micPio, g_micSm, true);
}

// Index des nächsten Worts, das die DMA schreiben wird.
static inline uint32_t mic_write_idx() {
    uint32_t addr = (uint32_t)dma_channel_hw_addr((uint)g_micDmaChannel)->write_addr;
    return ((addr - (uint32_t)g_micRing) / sizeof(uint32_t)) & kMicRingMask;
}

static inline uint32_t mic_available() {
    return (mic_write_idx() - g_micReadIdx) & kMicRingMask;
}

// Wartet, bis kMicTargetFill Samples im Ring liegen, und setzt den
// Lesezeiger an den Anfang dieses Vorlaufs. Einmal vor der Audio-
// Schleife aufrufen. Die ersten ~85 ms liefert das INMP441 ohnehin
// noch keine gültigen Daten.
void mic_prefill() {
    g_micReadIdx = mic_write_idx();
    while (mic_available() < kMicTargetFill) tight_loop_contents();
}

// Liefert n Mic-Samples als Q16 (DC-bereinigt, mit kMicGainShift
// skaliert). Aufbau eines DMA-Worts (MSB zuerst eingeschoben):
//   Bit 31     : I2S-Verzögerungsbit (kein Datenbit, per Pulldown 0)
//   Bit 30..7  : 24 Datenbits D23..D0
//   Bit 6..0   : Füllbits
void mic_read_block(q16 *out, uint32_t n) {
    if (!dma_channel_is_busy((uint)g_micDmaChannel)) {
        dma_channel_set_trans_count((uint)g_micDmaChannel, 0xFFFFFFFFu, true);
    }

    uint32_t avail = mic_available();
    if (avail > kMicMaxFill) {
        // Zu viel Vorlauf (Mic minimal schneller als DAC, oder lange
        // Unterbrechung) -> auf Sollvorlauf zurückspringen.
        g_micReadIdx = (mic_write_idx() - kMicTargetFill) & kMicRingMask;
        ++g_micOverruns;
    } else if (avail < n) {
        // Zu wenig da (Mic minimal langsamer als DAC) -> kurz warten.
        ++g_micUnderruns;
        while (mic_available() < n) tight_loop_contents();
    }

    for (uint32_t i = 0; i < n; ++i) {
        uint32_t raw = g_micRing[(g_micReadIdx + i) & kMicRingMask];
        int32_t s24 = (int32_t)(raw << 1) >> 8;            // Verzögerungsbit raus, Vorzeichen erhalten
        q16 x = (q16)((s24 * (1 << kMicGainShift)) >> 7);  // 24 Bit -> Q16 mit Gain
        g_micDcState = g_micDcState + q16_mul(g_micDcUpdateRate, x - g_micDcState);
        out[i] = x - g_micDcState;
    }
    g_micReadIdx = (g_micReadIdx + n) & kMicRingMask;
}

uint32_t mic_fill()      { return mic_available(); }
uint32_t mic_underruns() { return g_micUnderruns; }
uint32_t mic_overruns()  { return g_micOverruns; }
q16      mic_dc_level()  { return g_micDcState; }