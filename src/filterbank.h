#pragma once
// =====================================================================
// filterbank.h - 10-Band-Filterbank (Analyse + Synthese + VCA)
// =====================================================================
// Nachbau der Software-Referenz (modular-synth, VocoderBands.ts): Bänder,
// Frequenzen und Q stehen in config.h, die Bandgewichtung in
// filterbank.cpp. Die Arbeit wird pro Puffer auf zwei Kerne verteilt
// (dual_core.h); filterbank_process() rechnet einen Bereich von Bändern.

#include <cstdint>
#include "config.h"
#include "fixed_point.h"

// Diagnose pro Band. Jedes Band wird nur von "seinem" Kern beschrieben;
// lesen/zurücksetzen nur, während Core1 gerade NICHT rechnet.
struct FilterbankDiag {
    q16 bandMax[kNumBands] = {};    // Maximum der Analyse-Hüllkurve
    int64_t bandSum[kNumBands] = {}; // Summe der Analyse-Hüllkurve (-> bandsAvg)
    int64_t outSum[kNumBands] = {};  // Summe |Bandausgang| (-> outAvg)

    void reset() {
        for (int b = 0; b < kNumBands; ++b) {
            bandMax[b] = 0;
            bandSum[b] = 0;
            outSum[b] = 0;
        }
    }
};
extern FilterbankDiag g_filterbankDiag;

// Bänder, Synthesefilter und Bandgewichtung einrichten.
void filterbank_init();

// Bänder [firstBand, endBand) für einen ganzen Puffer rechnen.
//   modulator: vorverzerrtes Mic-Signal (Q16), carrier: Carrier (Q16)
//   mixOut:    Summe der Bandausgänge dieses Bereichs pro Sample
//   gateDetOut: Summe der Analyse-Hüllkurven dieses Bereichs pro Sample
void filterbank_process(int firstBand, int endBand,
                        const q16 *modulator, const q16 *carrier,
                        q16 *mixOut, q16 *gateDetOut);