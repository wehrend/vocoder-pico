#pragma once
#include <cstdint>
#include <cmath>

// Q16.16 Fixed-Point: 32-Bit signed Integer, 16 Integer-Bits,
// 16 Nachkommabits. Wertebereich ca. +-32767, Aufloesung ~1.5e-5.
// Fuer Audiosignale (~+-1.0) und Filterkoeffizienten (a1 kann bis an
// +-2.0 heranreichen) mehr als ausreichend Spielraum, ohne dass man
// wie bei Q1.15/Q1.31 auf die Koeffizientenreichweite aufpassen muesste.
//
// WARUM UEBERHAUPT: siehe DEVLOG - die 12-Band-Filterbank war in float
// auf dem FPU-losen RP2040 um Faktor ~8.6x zu langsam fuer 44.1kHz.
// Jede float-Operation landet dort als Aufruf einer Software-
// Bibliotheksroutine (__aeabi_fmul etc.) - das aendert sich nicht durch
// Compiler-Optimierung (-O3 brachte gemessen NICHTS), weil diese
// Routinen fertige Bibliotheksfunktionen sind. Ganzzahl-Multiplikation
// ist dagegen native, schnelle Hardware auf dem M0+.
using q16 = int32_t;

constexpr int kQ16Frac = 16;
constexpr q16 kQ16One = (q16)1 << kQ16Frac;

// NUR fuer einmalige Initialisierung (Koeffizienten/Konstanten
// berechnen) gedacht - nutzt float + lroundf, darf NICHT im
// Sample-Hot-Path aufgerufen werden, sonst ist der ganze Umbau umsonst.
inline q16 float_to_q16(float f) {
    return (q16)lroundf(f * (float)kQ16One);
}

inline float q16_to_float(q16 q) {
    return (float)q / (float)kQ16One;
}

// Q16.16 x Q16.16 -> Q16.16. Das Zwischenergebnis zweier 32-Bit-Werte
// kann bis zu 64 Bit breit sein, deshalb int64_t. Der M0+ hat dafuer
// keine Ein-Takt-Hardware (anders als die einfache 32x32->32-
// Multiplikation), der Compiler synthetisiert das ueber eine kompakte
// Laufzeit-Routine (~__aeabi_lmul) - immer noch um ein Vielfaches
// billiger als eine volle IEEE754-Software-Float-Multiplikation mit
// Exponentenausgleich/Normalisierung/Rundung.
inline q16 q16_mul(q16 a, q16 b) {
    return (q16)(((int64_t)a * (int64_t)b) >> kQ16Frac);
}
