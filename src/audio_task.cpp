// =====================================================================
// audio_task.cpp - Audio-Ablauf pro Puffer
// =====================================================================
// Ausgelagert aus main.cpp, Ablauf unverändert:
//   0) Pot lesen, Tonhöhe/Wellenform aus den Queues übernehmen,
//      freien DAC-Puffer holen, Mic-Block lesen
//   1) Vorbereitung (Core0, seriell): Rumpelfilter, Höhenanhebung,
//      Stimmhaft/Stimmlos, Carrier
//   2) Filterbank auf beiden Kernen
//   3) Kompressor, Gate, Ausgabe an den DAC
// Was pro Sample läuft, ist inline aus den Modul-Headern eingebettet.

#include "audio_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "pico/stdlib.h"
#include "pico/audio_i2s.h"

#include "config.h"
#include "fixed_point.h"
#include "mic_input.h"
#include "audio_output.h"
#include "controls.h"
#include "carrier_wave.h"
#include "carrier.h"
#include "voiced_unvoiced.h"
#include "dynamics.h"
#include "filterbank.h"
#include "dual_core.h"
#include "diagnostics.h"

namespace {

// Signalverarbeitungs-Module (Zustand; Parameter in den jeweiligen .cpp).
CarrierGenerator g_carrier;
VoicedUnvoiced g_vuv;
Compressor g_comp;
NoiseGate g_gate;
OutputLowpass g_outLowpass;

// Puffer pro Durchlauf: Modulator (Mic, nach Höhenanhebung), Carrier,
// und die über beide Kerne zusammengeführten Ergebnisse der Filterbank.
q16 sMicBlock[kBufferSamples];
q16 g_carrierBlock[kBufferSamples];
q16 g_mixBlock[kBufferSamples];
q16 g_gateDetBlock[kBufferSamples];

} // namespace

void audioTask(void *) {
    g_carrier.init();
    filterbank_init();
    g_vuv.init();

    g_comp.init();
    g_gate.init();
    g_outLowpass.init();

    // Reihenfolge wichtig: setup_audio() belegt DMA-Kanal 0 und pio0 für
    // den DAC, setup_mic() holt sich danach freie Ressourcen (pio1,
    // nächster freier DMA-Kanal).
    audio_buffer_pool_t *pool = setup_audio();
    setup_adc();
    setup_mic();
    // Core1 starten, bevor das Mic vorgefüllt wird (Filter sind schon
    // initialisiert). Läuft ab hier in dual_core.cpp und wartet.
    dual_core_start();
    mic_prefill();

    // Ein Puffer Mic-Samples pro Ausgabepuffer, vor der Sample-Schleife
    // komplett aus dem Ring geholt.

    float carrierHz = 220.0f;

    static Diagnostics diag;

    for (;;) {
        float formantPot = read_formant_pot();
        xQueueOverwrite(g_formantRawQueue, &formantPot);
        // Tonhöhe: eigenes Poti nur wenn aktiviert, sonst fest.
        float potNorm = formantPot;            // für die Diagnose ("pot=")
        if (kPitchPotEnabled) {
            potNorm = read_pot();
        }
        static float sFormantShift = 0.0f;
        xQueueReceive(g_formantShiftQueue, &sFormantShift, 0);
        if (kPitchPotEnabled) xQueueOverwrite(g_potRawQueue, &potNorm);
        if (kPitchPotEnabled) {
            xQueueReceive(g_carrierFreqQueue, &carrierHz, 0);
        } else {
            carrierHz = kMinCarrierHz;         // feste Tonhöhe
        }
        // Wellenwechsel nur an Puffergrenzen übernehmen; im ersten Puffer
        // nach dem Wechsel wird von der alten zur neuen Form übergeblendet.
        static uint8_t sWave = kWaveImpulse;
        static uint8_t sPrevWave = kWaveImpulse;
        sPrevWave = sWave;
        xQueueReceive(g_waveQueue, &sWave, 0);

        uint64_t waitStartUs = time_us_64();
        audio_buffer_t *buf = take_audio_buffer(pool, true);
        uint64_t waitUs = time_us_64() - waitStartUs;
        int16_t *samples = (int16_t *)buf->buffer->bytes;

        // Mic-Wartezeit getrennt messen, damit avg wieder die echte
        // Rechenlast zeigt (vorher steckte das Warten aufs Mic mit drin).
        uint64_t micStartUs = time_us_64();
        mic_read_block(sMicBlock, kBufferSamples);
        uint64_t loopStartUs = time_us_64();
        uint32_t micWaitUs = (uint32_t)(loopStartUs - micStartUs);

        // 1) Carrier-Block für den ganzen Puffer vorbereiten (Phase,
        //    Rausch-Formung - zustandsbehaftet, darum nur auf Core0).
        uint64_t prepStartUs = time_us_64();
        g_carrier.begin_buffer(carrierHz, sWave, sPrevWave);
        for (uint32_t i = 0; i < kBufferSamples; ++i) {
            q16 micQ16 = sMicBlock[i];
            diag.mic_sample(micQ16);

            // Rumpelfilter + Höhenanhebung: ab hier ist das der Modulator
            // für die Analyse auf beiden Kernen. Dazu Stimmhaft/Stimmlos.
            bool isUnvoiced;
            sMicBlock[i] = g_vuv.process(micQ16, isUnvoiced);
            if (isUnvoiced) diag.unvoiced_sample();

            // Carrier: stimmhaft (gewählte Wellenform) <-> Rauschen.
            q16 voicedQ16, unvoicedQ16;
            g_carrier.next(i, voicedQ16, unvoicedQ16);
            g_carrierBlock[i] = g_vuv.crossfade(voicedQ16, unvoicedQ16);
        }
        uint32_t prepUs = (uint32_t)(time_us_64() - prepStartUs);

        // 2) Filterbank auf beiden Kernen (Core0: Bänder 0..4, Core1: 5..9).
        filterbank_set_formant_shift(sFormantShift);   // Core1 rechnet gerade nicht
        DualCoreTiming timing = dual_core_process(sMicBlock, g_carrierBlock,
                                                  g_mixBlock, g_gateDetBlock);

        // 3) Kompressor, Gate, Ausgabe.
        for (uint32_t i = 0; i < kBufferSamples; ++i) {
            q16 mixed = g_mixBlock[i];
            q16 gateDetRaw = g_gateDetBlock[i];
            q16 gateDetector = g_gate.update_detector(gateDetRaw);
            diag.gate_detector(gateDetector);

            mixed = g_comp.process(mixed, i);   // Level -> Kompressor -> Makeup
            mixed = g_gate.apply(mixed);        // Noise-Gate mit Hysterese
            mixed = g_outLowpass.process(mixed);

            if (mixed > kQ16One) mixed = kQ16One;
            if (mixed < -kQ16One) mixed = -kQ16One;

            int32_t s32 = (int32_t)(((int64_t)mixed * 32767) >> kQ16Frac);
            if (s32 > 32767) s32 = 32767;
            if (s32 < -32768) s32 = -32768;
            int16_t s = (int16_t)s32;
            samples[2 * i]     = s;
            samples[2 * i + 1] = s;
        }

        buf->sample_count = buf->max_sample_count;
        give_audio_buffer(pool, buf);

        uint64_t elapsedUs = time_us_64() - loopStartUs;
        BufferStats stats;
        stats.elapsedUs = (uint32_t)elapsedUs;
        stats.waitUs = (uint32_t)waitUs;
        stats.micWaitUs = micWaitUs;
        stats.prepUs = prepUs;
        stats.cores = timing;
        stats.potNorm = potNorm;
        stats.carrierHz = carrierHz;
        stats.wave = sWave;
        stats.formantShift = sFormantShift;
        diag.buffer_done(stats, g_comp, g_gate);

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