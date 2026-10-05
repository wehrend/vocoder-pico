#pragma once
// =====================================================================
// carrier_wave.h - Auswahl der Carrier-Wellenform
// =====================================================================
// Gemeinsam genutzt von controls (Taster/LED) und der Carrier-Erzeugung
// im audioTask. Pegel- und Spektralabgleich der Wellenformen: siehe
// Kommentar "CARRIER-WELLENFORMEN" in main.cpp und DEVLOG.

#include <cstdint>

enum CarrierWave : uint8_t {
    kWaveImpulse = 0,
    kWaveSaw     = 1,
    kWaveSquare  = 2,
    kWaveNoise   = 3,   // nur Rauschen -> Flüsterstimme
    kWaveCount   = 4
};
inline const char *const kWaveNames[kWaveCount] = {"Impulszug", "Saegezahn", "Rechteck", "Rauschen"};

// KALIBRIER-SCHALTER: -1 = Wellenform per Taster wählbar (Normalbetrieb).
// 0..3 = fest eingestellt, Taster wird ignoriert:
//   0 = Impulszug, 1 = Sägezahn, 2 = Rechteck, 3 = Rauschen
constexpr int kFixedWave = -1;