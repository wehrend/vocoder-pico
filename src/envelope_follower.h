#pragma once
#include "biquad_fixed.h"
#include "fixed_point.h"
#include <cmath>

// Einzelner Envelope-Follower fuer das vereinfachte Ziel (siehe DEVLOG,
// "kleinere Broetchen" nach der 12-Band-Vocoder-Serie): Bandpass
// (fokussiert auf sprachrelevanten Bereich, unterdrueckt DC/Rumpeln)
// -> Gleichrichtung -> Attack/Release-Huellkurve. Anders als beim
// Filterbank-Vocoder gibt es KEINEN Synthese-Bandpass mehr - der
// Carrier wird direkt mit dieser EINEN Huellkurve multipliziert
// (reine Amplitudenmodulation, keine spektrale Formung mehr).
struct EnvelopeFollower {
    BiquadFixed bandpass;
    q16 envelope = 0;
    q16 attackCoeff = 0;
    q16 releaseCoeff = 0;

    void init(float freqHz, float q, float attackMs, float releaseMs, float sampleRate) {
        bandpass.setBandpass(freqHz, q, sampleRate);
        attackCoeff  = float_to_q16(expf(-1.0f / (0.001f * attackMs  * sampleRate)));
        releaseCoeff = float_to_q16(expf(-1.0f / (0.001f * releaseMs * sampleRate)));
    }

    inline q16 process(q16 modulatorSample) {
        q16 filtered = bandpass.process(modulatorSample);
        q16 rectified = (filtered < 0) ? -filtered : filtered;
        q16 coeff = (rectified > envelope) ? attackCoeff : releaseCoeff;
        // Algebraisch identisch zu "coeff*envelope + (1-coeff)*rectified",
        // aber nur 1 statt 2 Multiplikationen (siehe biquad_fixed.h/
        // vocoder_band_fixed.h fuer dieselbe Herleitung).
        envelope = envelope + q16_mul(kQ16One - coeff, rectified - envelope);
        return envelope;
    }
};
