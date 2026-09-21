#pragma once
#include "fixed_point.h"
#include <cmath>

// Fixed-Point-Pendant zu biquad.h. biquad.h bleibt unveraendert im
// Projekt liegen (als float-Referenz/Baseline, siehe DEVLOG-Messung) -
// main.cpp nutzt jetzt aber diese Variante im Sample-Hot-Path.
//
// WICHTIG: setBandpass() rechnet weiterhin bewusst in float
// (sinf/cosf/Division). Das laeuft nur EINMALIG beim Start pro Band,
// nicht pro Sample - dort spielt die float-Kosten keine Rolle, und
// float ist hier genauer und einfacher lesbar als eine Fixed-Point-
// Trig-Berechnung. Nur process() - der Teil, der 44100x pro Sekunde
// pro Band laeuft - ist reine Ganzzahl-Mathe.
// WICHTIG - SPEZIALISIERT AUF UNSER BANDPASS-DESIGN: b1 wird bewusst
// NICHT gespeichert (strukturell immer 0, siehe unten), UND b2 wird
// nicht separat gespeichert, weil bei der RBJ-Cookbook-Formel fuer
// einen Bandpass mit konstantem 0dB-Peak-Gain gilt: b2 == -b0, IMMER
// exakt (nicht nur naeherungsweise). b2*x laesst sich also aus dem
// bereits berechneten b0*x per Vorzeichenwechsel gewinnen - eine
// Multiplikation gespart, ohne jede Praezisionseinbusse. Falls dieser
// Code je fuer einen anderen Filtertyp (Tiefpass, Hochpass, Shelving,
// ...) wiederverwendet werden soll, gelten b1==0 und b2==-b0 NICHT
// mehr automatisch - siehe biquad.h (float-Version) fuer die
// vollstaendige, allgemeine Direct-Form-I-Gleichung.
struct BiquadFixed {
    q16 b0 = 0, a1 = 0, a2 = 0;
    q16 z1 = 0, z2 = 0;

    void setBandpass(float freqHz, float q, float sampleRate) {
        float w0 = 2.0f * (float)M_PI * freqHz / sampleRate;
        float alpha = sinf(w0) / (2.0f * q);
        float cosw0 = cosf(w0);
        float a0 = 1.0f + alpha;

        b0 = float_to_q16(alpha / a0);
        a1 = float_to_q16((-2.0f * cosw0) / a0);
        a2 = float_to_q16((1.0f - alpha) / a0);
    }

    inline q16 process(q16 x) {
        q16 b0x = q16_mul(b0, x);   // b2*x = -b0x, siehe Kommentar oben
        q16 y = b0x + z1;
        z1 = z2 - q16_mul(a1, y);
        z2 = -b0x - q16_mul(a2, y);
        return y;
    }
};
