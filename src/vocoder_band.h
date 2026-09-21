#pragma once
#include "biquad.h"
#include <cmath>

// Ein einzelnes Vocoder-Band: je ein Bandpass für Analyse (Modulator/Mic)
// und Synthese (Carrier), plus eine gemeinsame Hüllkurve.
//
// Analyse und Synthese brauchen ZWEI SEPARATE Biquad-Instanzen, obwohl
// beide dieselbe Mittenfrequenz/Q haben: jede Biquad-Instanz trägt eigenen
// Filterzustand (z1/z2), und Modulator- und Carrier-Signal laufen parallel
// durch die Bänder - ein gemeinsam genutztes Filter würde die Zustände
// beider Signale vermischen.
struct VocoderBand {
    Biquad analysisFilter;   // filtert das Mic-Signal (Modulator)
    Biquad synthesisFilter;  // filtert das Carrier-Signal
    float envelope = 0.0f;
    float attackCoeff = 0.0f;
    float releaseCoeff = 0.0f;

    void init(float freqHz, float q, float attackMs, float releaseMs, float sampleRate) {
        analysisFilter.setBandpass(freqHz, q, sampleRate);
        synthesisFilter.setBandpass(freqHz, q, sampleRate);
        attackCoeff  = expf(-1.0f / (0.001f * attackMs  * sampleRate));
        releaseCoeff = expf(-1.0f / (0.001f * releaseMs * sampleRate));
    }

    // Ein Modulator-Sample (Mic) verarbeiten und die Hüllkurve dieses
    // Bands aktualisieren. Muss PRO SAMPLE aufgerufen werden (siehe
    // main.cpp) - genau wie beim 1-Band-Envelope-Follower vorher.
    inline void analyze(float modulatorSample) {
        float filtered = analysisFilter.process(modulatorSample);
        float rectified = fabsf(filtered);
        float coeff = (rectified > envelope) ? attackCoeff : releaseCoeff;
        envelope = coeff * envelope + (1.0f - coeff) * rectified;
    }

    // Ein Carrier-Sample durch das Synthese-Bandpass dieses Bands filtern
    // und mit der zuletzt berechneten Hüllkurve skalieren. analyze() für
    // dasselbe Sample muss VORHER aufgerufen worden sein, damit envelope
    // aktuell ist.
    inline float synthesize(float carrierSample) {
        float filtered = synthesisFilter.process(carrierSample);
        return filtered * envelope;
    }
};

// Frequenzen logarithmisch (= konstante relative Bandbreite bei
// konstantem Q) zwischen freqLowHz und freqHighHz verteilen und alle
// Bänder initialisieren. Läuft einmalig beim Start, powf() pro Band ist
// hier unkritisch (kein Hot-Path).
inline void init_vocoder_bands(VocoderBand *bands, int numBands,
                                float freqLowHz, float freqHighHz,
                                float q, float attackMs, float releaseMs,
                                float sampleRate) {
    for (int i = 0; i < numBands; ++i) {
        float t = (numBands == 1) ? 0.0f : (float)i / (float)(numBands - 1);
        float freq = freqLowHz * powf(freqHighHz / freqLowHz, t);
        bands[i].init(freq, q, attackMs, releaseMs, sampleRate);
    }
}
