#pragma once
// =====================================================================
// filterbank.h - 10-Band-Filterbank (Analyse + Synthese + VCA)
// =====================================================================
// Nachbau der Software-Referenz (modular-synth, VocoderBands.ts): Bänder,
// Frequenzen und Q stehen in config.h, die Bandgewichtung in
// filterbank.cpp.
//
// ZWEI PHASEN pro Puffer (für den Bändertausch, siehe DEVLOG):
//   1. filterbank_analyze():    Analysebänder -> Hüllkurven aller Bänder
//                               für den ganzen Puffer in einen gemeinsamen
//                               Speicher
//   2. filterbank_synthesize(): Synthesebänder; Band b nimmt die Hüllkurve
//                               des ZUGEORDNETEN Analysebands (Formant-
//                               Verschiebung)
// Beide Phasen werden auf beide Kerne verteilt (dual_core.h); zwischen
// den Phasen liegt ein Handshake, weil ein Syntheseband die Hüllkurve
// eines Analysebands vom anderen Kern brauchen kann.

#include <cstdint>
#include "config.h"
#include "fixed_point.h"

// Formant-Verschiebung: Bereich in Bändern (ein Band ~0.68 Oktaven).
// Positiv = Formanten nach oben (Stimme kleiner/heller), negativ =
// nach unten (größer/dunkler). Zwischen ganzen Bändern wird überblendet.
constexpr float kFormantMaxShiftBands = 3.0f;

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

// Formant-Verschiebung für den nächsten Puffer setzen (in Bändern,
// begrenzt auf ±kFormantMaxShiftBands). Nur aufrufen, während Core1
// NICHT rechnet (vor dual_core_process).
void filterbank_set_formant_shift(float shiftBands);

// Phase 1: Analysebänder [firstBand, endBand) für den ganzen Puffer.
//   modulator:  vorverzerrtes Mic-Signal (Q16)
//   gateDetOut: Summe der Analyse-Hüllkurven dieses Bereichs pro Sample
void filterbank_analyze(int firstBand, int endBand,
                        const q16 *modulator, q16 *gateDetOut);

// Phase 2: Synthesebänder [firstBand, endBand) für den ganzen Puffer.
//   carrier: Carrier (Q16); mixOut: Summe der Bandausgänge pro Sample
void filterbank_synthesize(int firstBand, int endBand,
                           const q16 *carrier, q16 *mixOut);