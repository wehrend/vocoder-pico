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
// GESTRICHEN gegenüber der ursprünglichen 12-Band-Vocoder-Version
// (nicht mehr gebraucht) - Stand nach dem Talkbox-Umbau, TEILWEISE
// durch den 3-Band-Ausbau (Nachtrag 44) wieder relativiert, siehe dort:
// - 12-Band-Filterbank -> jetzt wieder 3 Bänder (VocoderBandFixed),
//   aber bewusst nicht gleich wieder 12
// - Zwei-Kern-Aufteilung (Core1/pico_multicore) - Rechenlast bei 3
//   Bändern weiterhin trivial genug für einen einzelnen Kern
// - Stimmhaft/Unstimmhaft-Erkennung + Rauschmischung im Carrier - war
//   nur nötig, um unstimmhaften Lauten in der SPEKTRALEN Vocoder-
//   Verarbeitung Energie zu geben; für den 3-Band-Zwischenschritt noch
//   nicht wieder eingeführt (siehe Nachtrag 44)
// - Pre-Emphasis-Filter (Nachtrag 24-26) - war ein Versuch, den
//   analogen Bandbreiten-Verlust für die Vocoder-Formant-Erkennung
//   auszugleichen; bisher nicht wieder eingeführt
//
// BEHALTEN: Oszillator/Poti-Tonhöhe, DC-Tracking im Mic-Read,
// Kompressor, Noise-Gate (mit Hysterese), FreeRTOS-Grundgerüst
// (audioTask/controlTask, Priority-Starvation-Fix) - alles bereits
// einzeln verifiziert funktionierend, siehe DEVLOG.
//
// ABNAHMEKRITERIUM für das Talkbox-Zwischenziel (siehe DEVLOG): Ton
// wird beim Sprechen/Singen vor dem Mic hörbar lauter/leiser, bleibt
// bei Stille sauber still (Gate), verzerrt nicht bei normaler
// Sprechlautstärke (Kompressor). Erreicht und stabil (siehe Nachtrag
// 40), zunächst im eingegrenzten 180-300Hz-Bereich (Nachtrag 38/39),
// nach der Sternerdung (Nachtrag 47) über den vollen 80-400Hz-Bereich
// bestätigt stabil - siehe dort für die eigentliche Ursache
// (Masseschleife) und ihre Behebung.
//
// === NACHTRAG 44: AUSBAU AUF 3 BÄNDER (feature/multiband-vocoder) ===
// Erster schrittweiser Ausbau Richtung Mehrband-Vocoder, bewusst NICHT
// gleich auf 12 Bänder gesprungen (Lehre aus Nachtrag 27/34). Statt
// EINEM Bandpass + EINER Hüllkurve: 3 Bänder, jeweils mit eigenem
// Analyse-Bandpass (Mic) UND eigenem Synthese-Bandpass (Carrier) -
// das ist der eigentliche Unterschied zwischen Talkbox (reine
// Amplitudenmodulation) und Vocoder (spektrale Formung). Wiederverwendet
// `vocoder_band_fixed.h` unverändert aus dem 12-Band-Projekt. Noch
// KEINE Stimmhaft/Unstimmhaft-Erkennung, noch kein Ausbau der
// Bandzahl über 3 hinaus - erst dieser Schritt stabilisieren, dann
// nach demselben Muster weiter hochskalieren.

// === UMSTIEG AUF DAS INMP441 (digitales I2S-MEMS-Mikrofon) ===
// Der analoge Mic-Pfad (MAX4466 -> RP2040-ADC) ist komplett ersetzt.
// Zwei Gründe, siehe DEVLOG:
// 1. Der ADC wurde bisher in der Sample-Schleife per adc_read() gelesen,
//    also so schnell wie die Schleife lief (256/avg) statt mit
//    kSampleRateHz, und in Bursts mit Lücken bis zum nächsten I2S-Puffer.
//    Dadurch lagen alle Analysebänder um den Faktor budget/avg höher als
//    berechnet, und jede Pufferlücke erzeugte einen Sprung im Signal.
//    -> Alle bisherigen Kalibrierungen (Bänder, Gewichtung, Gate,
//    Kompressor, Mic-Bandbreitenmessung) sind damit NEU zu machen.
// 2. Bandbreite/Störanfälligkeit des analogen Vorverstärkers.
//
// Neuer Pfad: PIO-I2S-Empfänger (i2s_mic.pio, auf pio1) -> DMA in einen
// Ringpuffer, hardware-getaktet mit exakt kSampleRateHz. Die Audio-
// Schleife holt pro I2S-Ausgabepuffer einen Block von kBufferSamples
// Mic-Samples aus dem Ring (mic_read_block). Getestet vorab isoliert
// im Projekt hello_mic_test.
//
// Der Pot bleibt am internen ADC (GP26) - nicht zeitkritisch.

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "pico/stdlib.h"
#include "pico/audio_i2s.h"
#include "pico/time.h"
#include "config.h"
#include "mic_input.h"
#include "audio_output.h"
#include "controls.h"
#include "carrier_wave.h"
#include "carrier.h"
#include "voiced_unvoiced.h"
#include "dynamics.h"
#include "filterbank.h"
#include "dual_core.h"
#include "fixed_point.h"
#include "biquad_fixed.h"
#include <cmath>
#include <cstdint>
#include <cstdio>

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
    // Drei einzelne Band-Hüllkurven statt einer aggregierten - hat sich
    // im 12-Band-Projekt (dortiger Nachtrag 22/23) als der Diagnose-
    // Wert herausgestellt, der tatsächlich zeigt, ob die Bänder
    // spektral differenzieren, statt nur die Gesamtlautstärke.
    // Array statt fester Einzelvariablen - wächst automatisch mit
    // kNumBands mit, kein Handeintrag mehr nötig beim nächsten
    // Hochskalieren (siehe DEVLOG).
    // Mittelwert der Band-Hüllkurven über das Diagnosefenster (Summe
    // über alle Samples) - für Messungen mit Rosa Rauschen / Stille /
    // Vokalen. Das Maximum oben springt bei Rauschen zu stark, um daraus
    // einen Frequenzgang oder Grundpegel abzulesen.
    static uint64_t sMicWaitSumUs = 0;
    // Zweikern-Diagnose: Rechenzeit der Core0-Bänder und wie lange Core0
    // danach noch auf Core1 warten musste (beides gemittelt pro Puffer).
    static uint64_t sCore0BandsSumUs = 0;
    static uint64_t sPrepSumUs = 0;      // Pre-Emphasis + Erkennung + Carrier (seriell, Core0)
    static uint32_t sUnvoicedSamples = 0; // Anteil "stimmlos" im Fenster
    static uint64_t sCore1ExtraWaitSumUs = 0;
    static int64_t sGateDetSum = 0;
    static q16 sGateDetMaxQ16 = 0;
    static int sLastPotPermille = 0;
    static int sLastCarrierHzInt = 0;
    static int sLastGateEnvPermille = 0;
    static int sLastGateGainPermille = 0;
    // NEU: zeigt den nachverfolgten Gleichspannungs-Arbeitspunkt des
    // Mic-Eingangs (g_micDcState) an - soll klären, ob sich der
    // Arbeitspunkt während einer Rückkopplungs-Episode verschiebt,
    // ohne dass dafür live am laufenden Gerät gemessen werden muss.
    static int sLastMicDcPermille = 0;
    // Mic-Ringpuffer: Füllstand (Soll kMicTargetFill) und Anzahl der
    // Korrekturen seit Start - sollten im Normalbetrieb 0 oder sehr
    // selten sein. Häufige Korrekturen = Schleife zu langsam bzw. Takte
    // driften stärker als erwartet.
    static uint32_t sLastMicFill = 0;

    for (;;) {
        float potNorm = read_pot();
        xQueueOverwrite(g_potRawQueue, &potNorm);
        xQueueReceive(g_carrierFreqQueue, &carrierHz, 0);
        // Wellenwechsel nur an Puffergrenzen übernehmen; im ersten Puffer
        // nach dem Wechsel wird von der alten zur neuen Form übergeblendet.
        static uint8_t sWave = kWaveImpulse;
        static uint8_t sPrevWave = kWaveImpulse;
        sPrevWave = sWave;
        xQueueReceive(g_waveQueue, &sWave, 0);

        uint64_t waitStartUs = time_us_64();
        audio_buffer_t *buf = take_audio_buffer(pool, true);
        uint64_t waitUs = time_us_64() - waitStartUs;
        sWaitSumUs += waitUs;
        if ((uint32_t)waitUs > sWaitMaxUs) sWaitMaxUs = (uint32_t)waitUs;
        int16_t *samples = (int16_t *)buf->buffer->bytes;

        sLastPotPermille = (int)(potNorm * 1000.0f);
        sLastCarrierHzInt = (int)carrierHz;
        // Mic-Wartezeit getrennt messen, damit avg wieder die echte
        // Rechenlast zeigt (vorher steckte das Warten aufs Mic mit drin).
        uint64_t micStartUs = time_us_64();
        mic_read_block(sMicBlock, kBufferSamples);
        uint64_t loopStartUs = time_us_64();
        uint32_t micWaitUs = (uint32_t)(loopStartUs - micStartUs);
        sMicWaitSumUs += micWaitUs;

        // 1) Carrier-Block für den ganzen Puffer vorbereiten (Phase,
        //    Rausch-Formung - zustandsbehaftet, darum nur auf Core0).
        uint64_t prepStartUs = time_us_64();
        g_carrier.begin_buffer(carrierHz, sWave, sPrevWave);
        for (uint32_t i = 0; i < kBufferSamples; ++i) {
            q16 micQ16 = sMicBlock[i];
            if (micQ16 < sMicMinQ16) sMicMinQ16 = micQ16;
            if (micQ16 > sMicMaxQ16) sMicMaxQ16 = micQ16;

            // Rumpelfilter + Höhenanhebung: ab hier ist das der Modulator
            // für die Analyse auf beiden Kernen. Dazu Stimmhaft/Stimmlos.
            bool isUnvoiced;
            sMicBlock[i] = g_vuv.process(micQ16, isUnvoiced);
            if (isUnvoiced) ++sUnvoicedSamples;

            // Carrier: stimmhaft (gewählte Wellenform) <-> Rauschen.
            q16 voicedQ16, unvoicedQ16;
            g_carrier.next(i, voicedQ16, unvoicedQ16);
            g_carrierBlock[i] = g_vuv.crossfade(voicedQ16, unvoicedQ16);
        }
        sPrepSumUs += (uint32_t)(time_us_64() - prepStartUs);

        // 2) Filterbank auf beiden Kernen (Core0: Bänder 0..4, Core1: 5..9).
        DualCoreTiming timing = dual_core_process(sMicBlock, g_carrierBlock,
                                                  g_mixBlock, g_gateDetBlock);
        sCore0BandsSumUs += timing.core0BandsUs;
        sCore1ExtraWaitSumUs += timing.core1ExtraWaitUs;

        // 3) Kompressor, Gate, Ausgabe.
        for (uint32_t i = 0; i < kBufferSamples; ++i) {
            q16 mixed = g_mixBlock[i];
            q16 gateDetRaw = g_gateDetBlock[i];
            q16 gateDetector = g_gate.update_detector(gateDetRaw);
            if (gateDetector > sGateDetMaxQ16) sGateDetMaxQ16 = gateDetector;
            sGateDetSum += gateDetector;

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
        sSumUs += elapsedUs;
        if ((uint32_t)elapsedUs > sMaxUs) sMaxUs = (uint32_t)elapsedUs;
        sLastGateEnvPermille = (int)(((int64_t)g_comp.envelope * 1000) / kQ16One);
        sLastGateGainPermille = (int)(((int64_t)g_gate.gain * 1000) / kQ16One);
        sLastMicDcPermille = (int)(((int64_t)mic_dc_level() * 1000) / kQ16One);
        sLastMicFill = mic_fill();

        if (++sBufferCount >= 100) {
            int micMinPermille = (int)(((int64_t)sMicMinQ16 * 1000) / kQ16One);
            int micMaxPermille = (int)(((int64_t)sMicMaxQ16 * 1000) / kQ16One);
            printf("Diagnose[%s %s]: avg=%luus max=%luus wait=%luus waitMax=%luus budget=%luus pot=%d/1000 carrierHz=%d micMin=%d/1000 micMax=%d/1000 micDc=%d/1000 compEnv=%d/1000 gateGain=%d/1000 (100 Puffer)\n",
                   __DATE__, __TIME__,
                   (unsigned long)(sSumUs / sBufferCount), (unsigned long)sMaxUs,
                   (unsigned long)(sWaitSumUs / sBufferCount), (unsigned long)sWaitMaxUs,
                   (unsigned long)kBufferBudgetUs,
                   sLastPotPermille, sLastCarrierHzInt, micMinPermille, micMaxPermille,
                   sLastMicDcPermille, sLastGateEnvPermille, sLastGateGainPermille);
            // Separate Zeile für die Band-Hüllkurven - Anzahl folgt
            // automatisch kNumBands, kein Umschreiben mehr nötig beim
            // nächsten Hochskalieren.
            printf("  mic: fill=%lu/%lu under=%lu over=%lu micWait=%luus\n",
                   (unsigned long)sLastMicFill, (unsigned long)kMicTargetFill,
                   (unsigned long)mic_underruns(), (unsigned long)mic_overruns(),
                   (unsigned long)(sMicWaitSumUs / sBufferCount));
            printf("  carrier: %s\n", kWaveNames[sWave]);
            printf("  vuv: stimmlos=%lu%% prep=%luus\n",
                   (unsigned long)((uint64_t)sUnvoicedSamples * 100 / ((uint64_t)sBufferCount * kBufferSamples)),
                   (unsigned long)(sPrepSumUs / sBufferCount));
            printf("  cores: core0Bands=%luus core1Warten=%luus (Core1 rechnet Bänder %d..%d)\n",
                   (unsigned long)(sCore0BandsSumUs / sBufferCount),
                   (unsigned long)(sCore1ExtraWaitSumUs / sBufferCount),
                   kCore1FirstBand, kNumBands - 1);
            {
                const int64_t kSamplesInWin = (int64_t)sBufferCount * kBufferSamples;
                float compMinGain = (float)g_comp.minGainSeen / (float)kQ16One;
                int compMinDb = (compMinGain > 0.0f) ? (int)(20.0f * log10f(compMinGain)) : -999;
                printf("  gateDet: avg=%d max=%d (x/10000, open=%d close=%d)  comp: maxReduktion=%ddB\n",
                       (int)((sGateDetSum * 10000) / ((int64_t)kQ16One * kSamplesInWin)),
                       (int)(((int64_t)sGateDetMaxQ16 * 10000) / kQ16One),
                       (int)(kGateOpenThreshold * 10000.0f), (int)(kGateCloseThreshold * 10000.0f),
                       compMinDb);
            }
            printf("  bands[0..%d]=", kNumBands - 1);
            for (int b = 0; b < kNumBands; ++b) {
                int bandPermille = (int)(((int64_t)g_filterbankDiag.bandMax[b] * 1000) / kQ16One);
                printf("%d ", bandPermille);
            }
            printf("\n");
            // Mittelwerte in 1/10000 (feiner als die Promille-Skala von
            // bands[] - leise Bänder liegen sonst alle bei 0-5).
            const int64_t kSamplesInWindow = (int64_t)sBufferCount * kBufferSamples;
            printf("  outAvg[0..%d] (x/1000)=", kNumBands - 1);
            for (int b = 0; b < kNumBands; ++b) {
                printf("%d ", (int)((g_filterbankDiag.outSum[b] * 1000) / ((int64_t)kQ16One * kSamplesInWindow)));
            }
            printf("\n");
            printf("  bandsAvg[0..%d] (x/10000)=", kNumBands - 1);
            for (int b = 0; b < kNumBands; ++b) {
                int avgPerTenThousand = (int)((g_filterbankDiag.bandSum[b] * 10000) / ((int64_t)kQ16One * kSamplesInWindow));
                printf("%d ", avgPerTenThousand);
            }
            printf("\n");
            sBufferCount = 0;
            sSumUs = 0;
            sMaxUs = 0;
            sWaitSumUs = 0;
            sWaitMaxUs = 0;
            sMicMinQ16 = kQ16One;
            sMicMaxQ16 = -kQ16One;
            g_filterbankDiag.reset();
            sMicWaitSumUs = 0;
            sCore0BandsSumUs = 0;
            sPrepSumUs = 0;
            sUnvoicedSamples = 0;
            sCore1ExtraWaitSumUs = 0;
            sGateDetSum = 0;
            sGateDetMaxQ16 = 0;
            g_comp.minGainSeen = kQ16One;
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

    controls_create_queues();

    // audioTask (Core0, hohe Priorität) verteilt die Bandberechnung pro
    // Puffer auf beide Kerne: Core1 läuft außerhalb von FreeRTOS in
    // core1_entry() und wird aus audioTask heraus gestartet.
    xTaskCreate(audioTask, "audio", 1024, nullptr, /*priority=*/3, nullptr);
    xTaskCreate(controlTask, "control", 512, nullptr, /*priority=*/1, nullptr);

    vTaskStartScheduler();

    panic("vTaskStartScheduler() zurueckgekehrt - Heap zu klein?");
    return 0;
}