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
// === PRÄZISIONS-UMBAU (siehe DEVLOG, Messung mit Rosa Rauschen) ===
// Vorher: Koeffizienten UND Filterzustand in Q16.16. Das reicht für
// laute Signale, aber bei leisen Signalen und tiefen Bändern nicht:
// Die Rundungsfehler in a1*y / a2*y werden von der Filterresonanz stark
// verstärkt (bei 90 Hz, Q=5 grob um den Faktor 1000). In der Simulation
// mit Rosa Rauschen bei RMS 0.01 zeigte das 90-Hz-Band dadurch 22/1000
// statt korrekter 2/1000 - fast exakt das Bild aus der Hardware.
//
// Jetzt:
//   Koeffizienten   Q2.30  (int32, Bereich ±2 - a1 liegt nahe -2)
//   Zustand/Ausgang Q8.24  (int32, Bereich ±128)
// Weiterhin nur 32x32->64-Bit-Multiplikationen, gleich viele wie vorher
// - praktisch keine Mehrkosten auf dem M0+. Simulation: deckungsgleich
// mit der float-Referenz bis hinunter zu RMS 0.003.
//
// BEREICH: Eingangssignal muss |x| < ~32 (Q16) bleiben, damit die
// Zustände (bis ~2-3x Eingangspegel) nicht über ±128 laufen. Mic bei
// kMicGainShift=3 liegt bei max. ±8, Carrier bei ±1 - reichlich Luft.
using q24 = int32_t;
using q30 = int32_t;
constexpr int kQ24Frac = 24;
constexpr int kQ30Frac = 30;

inline q30 float_to_q30(float f) {
    double v = (double)f * (double)(1ll << kQ30Frac);
    if (v >  2147483647.0) v =  2147483647.0;
    if (v < -2147483648.0) v = -2147483648.0;
    return (q30)llround(v);
}

inline q24 q30_mul_q24(q30 coeff, q24 x) {
    return (q24)(((int64_t)coeff * (int64_t)x) >> kQ30Frac);
}

struct BiquadFixed {
    q30 b0 = 0, a1 = 0, a2 = 0;
    q24 z1 = 0, z2 = 0;

    void setBandpass(float freqHz, float q, float sampleRate) {
        float w0 = 2.0f * (float)M_PI * freqHz / sampleRate;
        float alpha = sinf(w0) / (2.0f * q);
        float cosw0 = cosf(w0);
        float a0 = 1.0f + alpha;

        b0 = float_to_q30(alpha / a0);
        a1 = float_to_q30((-2.0f * cosw0) / a0);
        a2 = float_to_q30((1.0f - alpha) / a0);
    }

    // Kern: Eingang und Ausgang in Q8.24 (volle Auflösung, für die
    // Hüllkurve der Analyse-Seite).
    inline q24 processQ24(q24 x) {
        q24 b0x = q30_mul_q24(b0, x);   // b2*x = -b0x, siehe Kommentar oben
        q24 y = b0x + z1;
        z1 = z2 - q30_mul_q24(a1, y);
        z2 = -b0x - q30_mul_q24(a2, y);
        return y;
    }

    // Bisherige Schnittstelle (Q16 rein, Q16 raus) - z.B. Synthese-Seite.
    inline q16 process(q16 x) {
        return processQ24(x << (kQ24Frac - kQ16Frac)) >> (kQ24Frac - kQ16Frac);
    }
};
