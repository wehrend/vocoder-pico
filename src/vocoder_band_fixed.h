#pragma once
#include "biquad_fixed.h"
#include "fixed_point.h"
#include <cmath>

// Fixed-Point-Pendant zu vocoder_band.h - siehe dort fuer die
// ausfuehrliche Erklaerung von Analyse/Synthese-Aufteilung und warum
// zwei separate Biquad-Instanzen pro Band noetig sind (Filterzustand).
// Hier nur reine Ganzzahl-Mathe im Hot Path, Init weiterhin in float.
struct VocoderBandFixed {
    BiquadFixed analysisFilter;
    BiquadFixed synthesisFilter;
    q16 envelope = 0;
    q16 attackCoeff = 0;
    q16 releaseCoeff = 0;

    void init(float freqHz, float q, float attackMs, float releaseMs, float sampleRate) {
        analysisFilter.setBandpass(freqHz, q, sampleRate);
        synthesisFilter.setBandpass(freqHz, q, sampleRate);
        attackCoeff  = float_to_q16(expf(-1.0f / (0.001f * attackMs  * sampleRate)));
        releaseCoeff = float_to_q16(expf(-1.0f / (0.001f * releaseMs * sampleRate)));
    }

    inline void analyze(q16 modulatorSample) {
        q16 filtered = analysisFilter.process(modulatorSample);
        q16 rectified = (filtered < 0) ? -filtered : filtered;
        q16 coeff = (rectified > envelope) ? attackCoeff : releaseCoeff;
        // Algebraisch identisch zu "coeff*envelope + (1-coeff)*rectified",
        // aber nur 1 statt 2 Multiplikationen: envelope + (1-coeff)*(rectified-envelope)
        q16 oneMinusCoeff = kQ16One - coeff;
        q16 diff = rectified - envelope;
        envelope = envelope + q16_mul(oneMinusCoeff, diff);
    }

    inline q16 synthesize(q16 carrierSample) {
        q16 filtered = synthesisFilter.process(carrierSample);
        return q16_mul(filtered, envelope);
    }
};

inline void init_vocoder_bands_fixed(VocoderBandFixed *bands, int numBands,
                                      float freqLowHz, float freqHighHz,
                                      float q,
                                      float attackMsLow, float attackMsHigh,
                                      float releaseMsLow, float releaseMsHigh,
                                      float sampleRate) {
    for (int i = 0; i < numBands; ++i) {
        float t = (numBands == 1) ? 0.0f : (float)i / (float)(numBands - 1);
        float freq = freqLowHz * powf(freqHighHz / freqLowHz, t);
        // t=0 (tiefstes Band) -> langsame Zeitkonstanten (Formanten
        // bewegen sich langsam), t=1 (höchstes Band) -> schnelle
        // Zeitkonstanten (Konsonanten/Transienten brauchen das für
        // Verständlichkeit) - siehe DEVLOG für die Begründung.
        float attackMs = attackMsLow + t * (attackMsHigh - attackMsLow);
        float releaseMs = releaseMsLow + t * (releaseMsHigh - releaseMsLow);
        bands[i].init(freq, q, attackMs, releaseMs, sampleRate);
    }
}