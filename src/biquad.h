#pragma once
#include <cmath>

// Einfacher Biquad in Direct-Form-I (RBJ-Bandpass, konstanter 0dB-Peak-Gain).
// Ein Bandpass entfernt als Nebeneffekt auch den DC-Bias des Mic-Signals
// (MAX9814: 1.25V, MAX4466: ~VCC/2) - kein separates Zentrieren nötig.
struct Biquad {
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    float z1 = 0, z2 = 0;

    inline float process(float x) {
        float y = b0 * x + z1;
        z1 = b1 * x + z2 - a1 * y;
        z2 = b2 * x - a2 * y;
        return y;
    }

    void setBandpass(float f0, float Q, float sampleRate) {
        float w0 = 2.0f * (float)M_PI * f0 / sampleRate;
        float alpha = sinf(w0) / (2.0f * Q);
        float cosw0 = cosf(w0);

        float a0 = 1.0f + alpha;
        b0 = alpha / a0;
        b1 = 0.0f;
        b2 = -alpha / a0;
        a1 = (-2.0f * cosw0) / a0;
        a2 = (1.0f - alpha) / a0;
    }
};