// Talkbox-artiger Envelope-Follower auf generischem Raspberry Pi Pico +
// PCM5102A-Breakout - bewusst vereinfachtes Zwischenziel, nachdem die
// volle 12-Band-Filterbank (siehe DEVLOG bis Nachtrag 26) sich als zu
// große Baustelle auf einmal herausgestellt hat.
//
// PRINZIP, viel simpler als der Filterbank-Vocoder: EIN Bandpass +
// Gleichrichter + Attack/Release liefert EINE Gesamt-Hüllkurve aus dem
// Mic-Signal. Diese Hüllkurve moduliert direkt die Lautstärke eines
// Carrier-Oszillators (Pulszug, Tonhöhe per Poti) - reine Amplituden-
// modulation, KEINE spektrale Formung pro Frequenzband mehr. Kein
// Vocoder-Effekt (keine Formanten, keine erkennbaren Wörter), aber ein
// klar hörbares, reaktives Ergebnis: der Ton wird lauter/leiser im
// Rhythmus der Stimme - ein Talkbox-/Theremin-artiges Instrument.
//
// WARUM DAS EINE GUTE ZWISCHENSTATION IST: Es umgeht genau das
// Problem, an dem die Filterbank-Serie zuletzt hing (begrenzte
// Bandbreite des Mic-Vorverstärkers bei hohem Gain, siehe DEVLOG
// Nachtrag 23) - für eine reine Lautstärke-Hüllkurve ist es fast
// egal, ob hohe Frequenzen im Mic-Signal vollständig ankommen,
// Amplitude ist unabhängig vom Spektrum. Außerdem: die komplette
// Infrastruktur (Oszillator, Poti-Tonhöhe, DC-selbstkalibrierender
// Mic-Eingang, Kompressor, Noise-Gate) ist aus dem Vocoder-Projekt
// schon vorhanden und einzeln bewiesen funktionierend.
//
// GESTRICHEN gegenüber der Vocoder-Version (nicht mehr gebraucht):
// - 12-Band-Filterbank (VocoderBandFixed, vocoder_band_fixed.h)
// - Zwei-Kern-Aufteilung (Core1/pico_multicore) - bei nur einem
//   Envelope-Follower ist die Rechenlast trivial, ein Kern reicht
//   bequem
// - Stimmhaft/Unstimmhaft-Erkennung + Rauschmischung im Carrier - war
//   nur nötig, um unstimmhaften Lauten in der SPEKTRALEN Vocoder-
//   Verarbeitung Energie zu geben; für reine Amplitudenmodulation
//   irrelevant
// - Pre-Emphasis-Filter (Nachtrag 24-26) - war ein Versuch, den
//   analogen Bandbreiten-Verlust für die Vocoder-Formant-Erkennung
//   auszugleichen; hier nicht mehr relevant
//
// BEHALTEN: Oszillator/Poti-Tonhöhe, DC-Tracking im Mic-Read,
// Kompressor, Noise-Gate (mit Hysterese), FreeRTOS-Grundgerüst
// (audioTask/controlTask, Priority-Starvation-Fix) - alles bereits
// einzeln verifiziert funktionierend, siehe DEVLOG.
//
// ABNAHMEKRITERIUM für dieses Zwischenziel (siehe DEVLOG): Ton wird
// beim Sprechen/Singen vor dem Mic hörbar lauter/leiser, bleibt bei
// Stille sauber still (Gate), verzerrt nicht bei normaler
// Sprechlautstärke (Kompressor).

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "pico/stdlib.h"
#include "pico/audio_i2s.h"
#include "pico/time.h"
#include "hardware/adc.h"
#include "fixed_point.h"
#include "biquad_fixed.h"
#include "envelope_follower.h"
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {

// --- Pin-/ADC-Zuordnung: unverändert aus dem Vocoder-Projekt ---
constexpr uint POT_ADC_GPIO       = 26;
constexpr uint8_t POT_ADC_CHANNEL = 0;
constexpr uint MIC_ADC_GPIO       = 27;
constexpr uint8_t MIC_ADC_CHANNEL = 1;

// Carrier-Tonhöhenbereich - unverändert.
// ZURÜCKGESETZT auf 80Hz (siehe DEVLOG Nachtrag 31): die Einengung auf
// 150/200Hz war eine Reaktion auf Kontamination, die zumindest teilweise
// durch die (jetzt ebenfalls zurückgesetzte) 400Hz-Envelope-Analyse
// mitverursacht wurde, nicht nur durch die Carrier-Frequenz selbst.
// 80Hz war der zuletzt bestätigt funktionierende Wert, direkt nach der
// DIN-Leitungs-Umverlegung.
// Auf die bestätigt sichere Mitte eingegrenzt (war 80-400Hz, dann
// 180-380Hz) - siehe DEVLOG Nachtrag 38/39: 380Hz war immer noch zu
// nah am oberen Problembereich (355-379Hz zeigte anhaltendes
// Hängenbleiben, teils über mehrere Sekunden und mehrere Poti-
// Stellungen hinweg). 184Hz und 262-266Hz bestätigt sauber. Die
// eigentliche elektrische Ursache ist weiterhin nicht gefunden, das
// hier bleibt ein pragmatisches Eingrenzen, keine Lösung.
constexpr float kMinCarrierHz = 180.0f;
constexpr float kMaxCarrierHz = 300.0f;

// Rechenlast ist jetzt trivial (1 Bandpass statt 12) - volle 44.1kHz
// sind wieder problemlos drin, kein Grund mehr für die 22.05kHz-
// Absenkung aus der Vocoder-Serie.
constexpr uint32_t kSampleRateHz  = 44100;
constexpr uint32_t kBufferSamples = 256;

// --- Envelope-Follower-Parameter ---
// ZURÜCKGESETZT auf 1000Hz (siehe DEVLOG Nachtrag 31): die Senkung auf
// 400Hz hat vermutlich ein neues Problem erzeugt, das es vorher nicht
// gab - bei einem Poti-Bereich von 200-400Hz landet die 2. Harmonische
// eines Carriers (z.B. 224Hz -> 448Hz) fast direkt im 400Hz-Analyse-
// fenster, was das DAC->Mic-Übersprechen dort verstärkt durchschlagen
// lässt. 1000Hz war der zuletzt bestätigt funktionierende Wert.
constexpr float kEnvelopeFreqHz = 1000.0f;
constexpr float kEnvelopeQ      = 1.5f;
constexpr float kAttackMs       = 5.0f;
constexpr float kReleaseMs      = 100.0f;

EnvelopeFollower envelopeFollower;

// Pulszug-Wavetable für den Carrier (siehe DEVLOG Nachtrag 13 für die
// Begründung: gleichmäßigeres Obertonspektrum als ein Sägezahn). Hier
// nur noch fürs Timbre des Tons selbst relevant, nicht mehr für
// Formant-Abdeckung wie beim Vocoder.
constexpr int kCarrierTableSize = 512;
q16 carrierTable[kCarrierTableSize];

void build_carrier_table() {
    // 50% statt der ursprünglichen 20% (die waren fürs breitbandige
    // Obertonspektrum des 12-Band-Vocoders gedacht, siehe Nachtrag 13 -
    // für die Talkbox nicht mehr nötig). Ein Rechtecksignal hat von
    // Natur aus weniger hochfrequente Energie als ein schmaler
    // Pulszug - löst das "unangenehm schrill bei hohen Tonhöhen"-
    // Problem an der Quelle, statt nachträglich zu filtern (siehe
    // DEVLOG Nachtrag 36 - der Ausgangs-Tiefpass hatte einen
    // unerklärten Nebeneffekt auf die Gate-Rückkopplung).
    constexpr float kDutyCycle = 0.5f;
    for (int i = 0; i < kCarrierTableSize; ++i) {
        float phase = (float)i / (float)kCarrierTableSize;
        float pulse = (phase < kDutyCycle) ? 1.0f : -1.0f;
        carrierTable[i] = float_to_q16(pulse);
    }
}

audio_buffer_pool_t *setup_audio() {
    static audio_format_t audioFormat = {
        .sample_freq = kSampleRateHz,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2,
    };
    static audio_buffer_format_t producerFormat = {
        .format = &audioFormat,
        .sample_stride = 4,
    };

    audio_buffer_pool_t *pool = audio_new_producer_pool(&producerFormat, 4, kBufferSamples);

    audio_i2s_config_t i2sConfig = {
        .data_pin = PICO_AUDIO_I2S_DATA_PIN,
        .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
        .dma_channel = 0,
        .pio_sm = 0,
    };

    const audio_format_t *outputFormat = audio_i2s_setup(&audioFormat, &i2sConfig);
    if (!outputFormat) {
        panic("I2S-Setup fehlgeschlagen - Pins/Format pruefen");
    }

    audio_i2s_connect(pool);
    audio_i2s_set_enabled(true);
    return pool;
}

void setup_adc() {
    adc_init();
    adc_gpio_init(POT_ADC_GPIO);
    adc_gpio_init(MIC_ADC_GPIO);
}

float read_adc_normalized() {
    uint16_t raw = adc_read();
    return (float)raw / 4095.0f;
}

// Dynamisch nachverfolgter Gleichspannungs-Anteil des Mic-Signals,
// statt eines hart codierten angenommenen Mittelpunkts - siehe DEVLOG
// Nachtrag 17 (hat sich beim Vocoder-Projekt mehrfach als nötig
// erwiesen, sobald sich am analogen Signalweg etwas ändert).
constexpr float kMicDcTrackingMs = 300.0f;
q16 g_micDcState = 0;
q16 g_micDcUpdateRate = 0;

inline q16 read_adc_bipolar_q16() {
    int32_t raw = (int32_t)adc_read();
    q16 rawQ16 = raw << 5;
    g_micDcState = g_micDcState + q16_mul(g_micDcUpdateRate, rawQ16 - g_micDcState);
    return rawQ16 - g_micDcState;
}

// Queues zur Kommunikation zwischen audioTask (ADC-Besitzer) und
// controlTask (Glättung/Mapping) - unverändert aus dem Vocoder-Projekt.
QueueHandle_t g_potRawQueue = nullptr;
QueueHandle_t g_carrierFreqQueue = nullptr;

// --- Kompressor-Parameter (siehe DEVLOG Nachtrag 12 für die
// Herleitung) - Werte sind hier NEU zu kalibrieren, da die
// Signal-Skala ohne 12-fache Bandsummierung anders ist als beim
// Vocoder. Erster Schätzwert, kein gemessenes Optimum. ---
constexpr float kCompInputGain  = 2.0f;
constexpr float kCompThreshold  = 0.3f;
constexpr float kCompRatio      = 8.0f;
constexpr float kCompAttackMs   = 5.0f;
constexpr float kCompReleaseMs  = 150.0f;
constexpr float kCompMakeup     = 2.0f;

q16 g_compInputGainQ16 = 0;
q16 g_compThresholdQ16 = 0;
q16 g_compAttackCoeff = 0;
q16 g_compReleaseCoeff = 0;
q16 g_compMakeupQ16 = 0;
q16 g_compRatioInvQ16 = 0;
q16 g_compEnvelope = 0;

// --- Noise-Gate mit Hysterese (siehe DEVLOG Nachtrag 19/20/23 für die
// Herleitung, insbesondere warum Hysterese statt einer einzelnen
// Schwelle nötig war). Werte ebenfalls neu zu kalibrieren. ---
constexpr float kGateOpenThreshold  = 0.050f; // war 0.020 - Messung zeigte Ruhepegel bei 17-24/1000, direkt auf der alten Schwelle -> Geflacker, siehe DEVLOG
constexpr float kGateCloseThreshold = 0.030f; // war 0.008 - mit Abstand über dem beobachteten Ruhepegel
constexpr float kGateAttackMs   = 5.0f;
constexpr float kGateReleaseMs  = 40.0f; // war 120ms - vermutlich Hauptursache für hörbares Nachschwingen nach dem Sprechen, siehe DEVLOG

q16 g_gateOpenThresholdQ16 = 0;
q16 g_gateCloseThresholdQ16 = 0;
q16 g_gateAttackCoeff = 0;
q16 g_gateReleaseCoeff = 0;
q16 g_gateGain = 0;
bool g_gateIsOpen = false;

// Digitales Tiefpassfilter direkt am Ausgang, NACH Gate/Kompressor,
// VOR der int16-Wandlung - dämpft die scharfen oberen Harmonischen
// des schmalen Pulszug-Carriers (20% Duty-Cycle), die bei hohen
// Tonhöhen (nahe kMaxCarrierHz) als unangenehm hochfrequent/schrill
// auffielen (siehe DEVLOG). Bewusst SOFTWARE statt einer zusätzlichen
// Schaltung - reines Klangformungs-Problem, keine Rückwirkung auf die
// erst kürzlich stabilisierte Mic-/Gate-/Kompressor-Kette, und ohne
// das Risiko eines weiteren wackligen Steckbrett-Bauteils.
// kOutputLowpassFreqHz ist ein erster Schätzwert (dämpft grob ab dem
// Bereich, wo das Ohr am empfindlichsten auf "schrill" reagiert),
// kein gemessenes/gehörtes Optimum - nach Gehör nachjustieren.
constexpr float kOutputLowpassFreqHz = 3000.0f;
// DIAGNOSE-SCHALTER: false = Tiefpass komplett umgehen, um zu testen,
// ob er die Ursache für das beobachtete "Gate bleibt hängen" ist
// (Verdacht auf verstärkte DAC->Mic-Rückkopplung, siehe DEVLOG).
constexpr bool kOutputLowpassEnabled = false;
q16 g_outputLowpassCoeff = 0;
q16 g_outputLowpassState = 0;

void controlTask(void *) {
    float smoothedHz = 220.0f;
    for (;;) {
        float potNorm;
        if (xQueueReceive(g_potRawQueue, &potNorm, pdMS_TO_TICKS(50)) == pdTRUE) {
            float targetHz = kMinCarrierHz + potNorm * (kMaxCarrierHz - kMinCarrierHz);
            smoothedHz += (targetHz - smoothedHz) * 0.2f;
            xQueueOverwrite(g_carrierFreqQueue, &smoothedHz);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void audioTask(void *) {
    build_carrier_table();
    envelopeFollower.init(kEnvelopeFreqHz, kEnvelopeQ, kAttackMs, kReleaseMs, (float)kSampleRateHz);
    g_micDcUpdateRate = kQ16One - float_to_q16(expf(-1.0f / (0.001f * kMicDcTrackingMs * (float)kSampleRateHz)));

    g_compInputGainQ16 = float_to_q16(kCompInputGain);
    g_compThresholdQ16 = float_to_q16(kCompThreshold);
    g_compAttackCoeff = float_to_q16(expf(-1.0f / (0.001f * kCompAttackMs * (float)kSampleRateHz)));
    g_compReleaseCoeff = float_to_q16(expf(-1.0f / (0.001f * kCompReleaseMs * (float)kSampleRateHz)));
    g_compMakeupQ16 = float_to_q16(kCompMakeup);
    g_compRatioInvQ16 = float_to_q16(1.0f / kCompRatio);
    g_gateOpenThresholdQ16 = float_to_q16(kGateOpenThreshold);
    g_gateCloseThresholdQ16 = float_to_q16(kGateCloseThreshold);
    g_gateAttackCoeff = float_to_q16(expf(-1.0f / (0.001f * kGateAttackMs * (float)kSampleRateHz)));
    g_gateReleaseCoeff = float_to_q16(expf(-1.0f / (0.001f * kGateReleaseMs * (float)kSampleRateHz)));
    g_outputLowpassCoeff = float_to_q16(1.0f - expf(-2.0f * (float)M_PI * kOutputLowpassFreqHz / (float)kSampleRateHz));

    audio_buffer_pool_t *pool = setup_audio();
    setup_adc();

    uint32_t phase = 0;
    float carrierHz = 220.0f;

    // --- Diagnose-Zustand (persistiert über Puffergrenzen hinweg) ---
    constexpr uint32_t kBufferBudgetUs = (kBufferSamples * 1000000ull) / kSampleRateHz;
    static uint32_t sBufferCount = 0;
    static uint64_t sSumUs = 0;
    static uint32_t sMaxUs = 0;
    // NEU: I2S-Pufferwartezeit (take_audio_buffer) separat von der
    // reinen Sample-Verarbeitung gemessen - bisher war das in der
    // Diagnose gar nicht sichtbar. Soll klären, ob die beobachtete
    // Timing-Anomalie während der Rückkopplungs-Episoden aus dem
    // Warten auf den I2S-Puffer kommt (würde auf gestörte I2S-Taktung
    // hindeuten) oder aus der Sample-Schleife selbst (eher auf
    // Interrupt-Unterbrechungen hindeutend), siehe DEVLOG.
    static uint64_t sWaitSumUs = 0;
    static uint32_t sWaitMaxUs = 0;
    static q16 sMicMinQ16 = kQ16One;
    static q16 sMicMaxQ16 = -kQ16One;
    static q16 sEnvMaxQ16 = 0;
    static int sLastPotPermille = 0;
    static int sLastCarrierHzInt = 0;
    static int sLastGateEnvPermille = 0;
    static int sLastGateGainPermille = 0;
    // NEU: zeigt den nachverfolgten Gleichspannungs-Arbeitspunkt des
    // Mic-Eingangs (g_micDcState) an - soll klären, ob sich der
    // Arbeitspunkt während einer Rückkopplungs-Episode verschiebt,
    // ohne dass dafür live am laufenden Gerät gemessen werden muss.
    static int sLastMicDcPermille = 0;

    for (;;) {
        adc_select_input(POT_ADC_CHANNEL);
        float potNorm = read_adc_normalized();
        xQueueOverwrite(g_potRawQueue, &potNorm);
        xQueueReceive(g_carrierFreqQueue, &carrierHz, 0);
        uint32_t phaseInc = (uint32_t)((carrierHz * kCarrierTableSize / (float)kSampleRateHz) * 65536.0f);

        adc_select_input(MIC_ADC_CHANNEL);

        uint64_t waitStartUs = time_us_64();
        audio_buffer_t *buf = take_audio_buffer(pool, true);
        uint64_t waitUs = time_us_64() - waitStartUs;
        sWaitSumUs += waitUs;
        if ((uint32_t)waitUs > sWaitMaxUs) sWaitMaxUs = (uint32_t)waitUs;
        int16_t *samples = (int16_t *)buf->buffer->bytes;

        sLastPotPermille = (int)(potNorm * 1000.0f);
        sLastCarrierHzInt = (int)carrierHz;
        uint64_t loopStartUs = time_us_64();

        for (uint32_t i = 0; i < kBufferSamples; ++i) {
            q16 micQ16 = read_adc_bipolar_q16();
            if (micQ16 < sMicMinQ16) sMicMinQ16 = micQ16;
            if (micQ16 > sMicMaxQ16) sMicMaxQ16 = micQ16;

            q16 env = envelopeFollower.process(micQ16);
            if (env > sEnvMaxQ16) sEnvMaxQ16 = env;

            q16 carrierQ16 = carrierTable[(phase >> 16) & (kCarrierTableSize - 1)];
            q16 mixed = q16_mul(carrierQ16, env);

            // Level -> Kompressor -> Makeup statt hartem Clipping.
            mixed = q16_mul(mixed, g_compInputGainQ16);
            q16 absMixed = (mixed < 0) ? -mixed : mixed;
            q16 compCoeff = (absMixed > g_compEnvelope) ? g_compAttackCoeff : g_compReleaseCoeff;
            g_compEnvelope = g_compEnvelope + q16_mul(kQ16One - compCoeff, absMixed - g_compEnvelope);
            q16 compGain = kQ16One;
            if (g_compEnvelope > g_compThresholdQ16) {
                q16 excess = g_compEnvelope - g_compThresholdQ16;
                q16 compressedLevel = g_compThresholdQ16 + q16_mul(excess, g_compRatioInvQ16);
                compGain = (q16)(((int64_t)compressedLevel << kQ16Frac) / g_compEnvelope);
            }
            mixed = q16_mul(mixed, compGain);
            mixed = q16_mul(mixed, g_compMakeupQ16);

            // Noise-Gate mit Hysterese (Schmitt-Trigger-Muster).
            q16 gateThreshold = g_gateIsOpen ? g_gateCloseThresholdQ16 : g_gateOpenThresholdQ16;
            bool gateShouldBeOpen = (g_compEnvelope > gateThreshold);
            g_gateIsOpen = gateShouldBeOpen;
            q16 gateTarget = gateShouldBeOpen ? kQ16One : 0;
            q16 gateCoeff = (gateTarget > g_gateGain) ? g_gateAttackCoeff : g_gateReleaseCoeff;
            g_gateGain = g_gateGain + q16_mul(kQ16One - gateCoeff, gateTarget - g_gateGain);
            mixed = q16_mul(mixed, g_gateGain);

            // Ausgangs-Tiefpass gegen die scharfen Carrier-Obertöne
            // bei hohen Tonhöhen, siehe Erklärung oben.
            if (kOutputLowpassEnabled) {
                g_outputLowpassState = g_outputLowpassState +
                    q16_mul(g_outputLowpassCoeff, mixed - g_outputLowpassState);
                mixed = g_outputLowpassState;
            }

            if (mixed > kQ16One) mixed = kQ16One;
            if (mixed < -kQ16One) mixed = -kQ16One;

            int32_t s32 = (int32_t)(((int64_t)mixed * 32767) >> kQ16Frac);
            if (s32 > 32767) s32 = 32767;
            if (s32 < -32768) s32 = -32768;
            int16_t s = (int16_t)s32;
            samples[2 * i]     = s;
            samples[2 * i + 1] = s;
            phase += phaseInc;
        }

        buf->sample_count = buf->max_sample_count;
        give_audio_buffer(pool, buf);

        uint64_t elapsedUs = time_us_64() - loopStartUs;
        sSumUs += elapsedUs;
        if ((uint32_t)elapsedUs > sMaxUs) sMaxUs = (uint32_t)elapsedUs;
        sLastGateEnvPermille = (int)(((int64_t)g_compEnvelope * 1000) / kQ16One);
        sLastGateGainPermille = (int)(((int64_t)g_gateGain * 1000) / kQ16One);
        sLastMicDcPermille = (int)(((int64_t)g_micDcState * 1000) / kQ16One);

        if (++sBufferCount >= 100) {
            int micMinPermille = (int)(((int64_t)sMicMinQ16 * 1000) / kQ16One);
            int micMaxPermille = (int)(((int64_t)sMicMaxQ16 * 1000) / kQ16One);
            int envMaxPermille = (int)(((int64_t)sEnvMaxQ16 * 1000) / kQ16One);
            printf("Diagnose[%s %s]: avg=%luus max=%luus wait=%luus waitMax=%luus budget=%luus pot=%d/1000 carrierHz=%d micMin=%d/1000 micMax=%d/1000 micDc=%d/1000 envMax=%d/1000 gateEnv=%d/1000 gateGain=%d/1000 (100 Puffer)\n",
                   __DATE__, __TIME__,
                   (unsigned long)(sSumUs / sBufferCount), (unsigned long)sMaxUs,
                   (unsigned long)(sWaitSumUs / sBufferCount), (unsigned long)sWaitMaxUs,
                   (unsigned long)kBufferBudgetUs,
                   sLastPotPermille, sLastCarrierHzInt, micMinPermille, micMaxPermille,
                   sLastMicDcPermille, envMaxPermille, sLastGateEnvPermille, sLastGateGainPermille);
            sBufferCount = 0;
            sSumUs = 0;
            sMaxUs = 0;
            sWaitSumUs = 0;
            sWaitMaxUs = 0;
            sMicMinQ16 = kQ16One;
            sMicMaxQ16 = -kQ16One;
            sEnvMaxQ16 = 0;
        }

        // WICHTIG - PRIORITY-STARVATION-FIX (siehe DEVLOG Nachtrag 8):
        // audioTask hat sonst keinen echten FreeRTOS-Blockierpunkt,
        // controlTask bekäme ohne das nie CPU-Zeit.
        static uint32_t sYieldCounter = 0;
        if (++sYieldCounter >= 2) {
            sYieldCounter = 0;
            vTaskDelay(1);
        }
    }
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    panic("Stack-Overflow in Task: %s", pcTaskName);
}

extern "C" void vApplicationMallocFailedHook(void) {
    panic("FreeRTOS malloc fehlgeschlagen - configTOTAL_HEAP_SIZE in FreeRTOSConfig.h zu klein?");
}

} // namespace

int main() {
    stdio_init_all();

    g_potRawQueue = xQueueCreate(1, sizeof(float));
    g_carrierFreqQueue = xQueueCreate(1, sizeof(float));
    if (!g_potRawQueue || !g_carrierFreqQueue) {
        panic("Queue-Erstellung fehlgeschlagen (Heap zu klein?)");
    }

    // Rechenlast ist jetzt trivial (1 Envelope-Follower statt 12-Band-
    // Filterbank) - ein einzelner Task auf Core0 reicht bequem, kein
    // Core1 mehr nötig.
    xTaskCreate(audioTask, "audio", 1024, nullptr, /*priority=*/3, nullptr);
    xTaskCreate(controlTask, "control", 512, nullptr, /*priority=*/1, nullptr);

    vTaskStartScheduler();

    panic("vTaskStartScheduler() zurueckgekehrt - Heap zu klein?");
    return 0;
}