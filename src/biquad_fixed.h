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

// === Allgemeiner Biquad (für Shelf/Tiefpass/Hochpass des Stimmhaft/
// Stimmlos-Moduls, siehe main.cpp und modular-synth VoicedUnvoicedNode).
// Formeln wie Web Audio BiquadFilterNode (Audio-EQ-Cookbook), damit die
// Filter der Referenz exakt nachgebildet werden. Wichtig: Web Audio
// interpretiert Q bei Tief-/Hochpass in dB (Tone-Default Q=1 -> 10^(1/20)
// = 1.122 linear), bei Bandpass dagegen linear.
// Koeffizienten in Q3.29 (Bereich ±4 - Shelf-Filter brauchen b0 > 2),
// Zustand/Ein-/Ausgang in Q8.24 wie BiquadFixed. Transponierte
// Direktform II.
constexpr int kQ29Frac = 29;

inline int32_t float_to_q29(float f) {
    double v = (double)f * (double)(1ll << kQ29Frac);
    if (v >  2147483647.0) v =  2147483647.0;
    if (v < -2147483648.0) v = -2147483648.0;
    return (int32_t)llround(v);
}

inline q24 q29_mul_q24(int32_t coeff, q24 x) {
    return (q24)(((int64_t)coeff * (int64_t)x) >> kQ29Frac);
}

struct BiquadGeneralFixed {
    int32_t b0 = 0, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    q24 z1 = 0, z2 = 0;

    void setNormalized(float nb0, float nb1, float nb2, float na0, float na1, float na2) {
        b0 = float_to_q29(nb0 / na0);
        b1 = float_to_q29(nb1 / na0);
        b2 = float_to_q29(nb2 / na0);
        a1 = float_to_q29(na1 / na0);
        a2 = float_to_q29(na2 / na0);
    }

    // qDb: Q wie in Web Audio in dB (Tone.Filter-Default: 1).
    void setLowpass(float freqHz, float qDb, float sampleRate) {
        float w0 = 2.0f * (float)M_PI * freqHz / sampleRate;
        float cw = cosf(w0);
        float alpha = sinf(w0) / (2.0f * powf(10.0f, qDb / 20.0f));
        setNormalized((1.0f - cw) / 2.0f, 1.0f - cw, (1.0f - cw) / 2.0f,
                      1.0f + alpha, -2.0f * cw, 1.0f - alpha);
    }

    void setHighpass(float freqHz, float qDb, float sampleRate) {
        float w0 = 2.0f * (float)M_PI * freqHz / sampleRate;
        float cw = cosf(w0);
        float alpha = sinf(w0) / (2.0f * powf(10.0f, qDb / 20.0f));
        setNormalized((1.0f + cw) / 2.0f, -(1.0f + cw), (1.0f + cw) / 2.0f,
                      1.0f + alpha, -2.0f * cw, 1.0f - alpha);
    }

    // High-Shelf wie Web Audio (Steilheit S = 1, Q wird ignoriert).
    void setHighShelf(float freqHz, float gainDb, float sampleRate) {
        float A = powf(10.0f, gainDb / 40.0f);
        float w0 = 2.0f * (float)M_PI * freqHz / sampleRate;
        float cw = cosf(w0);
        float alpha = sinf(w0) / 2.0f * sqrtf(2.0f); // S = 1
        float sqA2a = 2.0f * sqrtf(A) * alpha;
        setNormalized(A * ((A + 1.0f) + (A - 1.0f) * cw + sqA2a),
                      -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cw),
                      A * ((A + 1.0f) + (A - 1.0f) * cw - sqA2a),
                      (A + 1.0f) - (A - 1.0f) * cw + sqA2a,
                      2.0f * ((A - 1.0f) - (A + 1.0f) * cw),
                      (A + 1.0f) - (A - 1.0f) * cw - sqA2a);
    }

    inline q24 processQ24(q24 x) {
        q24 y = q29_mul_q24(b0, x) + z1;
        z1 = q29_mul_q24(b1, x) - q29_mul_q24(a1, y) + z2;
        z2 = q29_mul_q24(b2, x) - q29_mul_q24(a2, y);
        return y;
    }

    inline q16 process(q16 x) {
        return processQ24(x << (kQ24Frac - kQ16Frac)) >> (kQ24Frac - kQ16Frac);
    }
};