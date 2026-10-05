// =====================================================================
// carrier.cpp - Carrier: Parameter, Pegelabgleich, Initialisierung
// =====================================================================
// Ausgelagert aus main.cpp, inhaltlich unverändert.
//
// Alle stimmhaften Carrier werden so angepasst, dass sie im Formant-
// bereich (Bänder 4..7, ~580-2400 Hz) dieselbe Leistung pro Band
// liefern wie das weiße Rauschen für die Zischlaute. Nur dann passt
// EINE Bandgewichtung (sqrt(365/f)) für alle Wellenformen, und Vokale
// und Zischlaute bleiben im Gleichgewicht (siehe DEVLOG: der rohe
// Sägezahn fiel um ~6 dB/Oktave ab, die hohen Formanten lagen 20-60 dB
// unter dem Grundton -> keine Vokalunterschiede hörbar).
//
// Impulszug: Ein Impuls der Höhe h pro Periode hat dieselbe Leistung pro
// Hz wie weißes Rauschen mit Standardabweichung sigma, wenn
//   h = sigma * sqrt(fs / f0)   (sigma = 1/sqrt(3) für Rauschen ±1).
// Simulation: Abweichung pro Band < 1 dB, bei jeder Tonhöhe.
//
// Sägezahn und Rechteck: Höhenanhebung y[n] = x[n] - a*x[n-1] mit
// a = kWavePreEmphasis, danach Skalierung * sqrt(110 Hz / f0).
// Simulation und Messung (DEVLOG, 5.10.): ab ~365 Hz innerhalb ±1 dB am
// Impulszug/Rauschen; die untersten Bänder behalten etwas mehr Bass -
// das ist der Charakter, der die Wellenformen dort unterscheidet.

#include "carrier.h"
#include <cmath>

namespace {
constexpr float kWavePreEmphasis  = 0.9f;
constexpr float kWaveRefHz        = 110.0f;
constexpr float kSawLevelAtRef    = 4.00f;  // Simulation + Messung: passt (Bänder 4..7 ±1.5 dB)
constexpr float kSquareLevelAtRef = 2.18f;  // war 2.81 - Messung: ~1.29x zu laut, danach ±1 dB
// Manueller Feinabgleich Vokale (stimmhaft) gegen Zischlaute (Rauschen).
constexpr float kVoicedLevel = 1.0f;
// Rauschen für stimmlose Laute: Referenz nutzt Tone.Noise (weiß, ~±1),
// next_noise() liefert ±0.5 -> x2.
constexpr float kUnvoicedNoiseGain = 2.0f;
// Alter fester Rauschanteil im stimmhaften Carrier, ersetzt durch die
// Stimmhaft/Stimmlos-Umschaltung; Pfad zum Vergleich erhalten.
constexpr float kCarrierNoiseMix  = 0.0f;
constexpr float kNoiseShapeFreqHz = 400.0f; // Tiefpass ("rosa") für diesen Pfad
} // namespace

void CarrierGenerator::init() {
    for (int i = 0; i < kTableSize; ++i) {
        float p = (float)i / (float)kTableSize;
        sawTable[i] = float_to_q16(2.0f * p - 1.0f);
    }
    preEmphasis = float_to_q16(kWavePreEmphasis);
    unvoicedNoiseGain = float_to_q16(kUnvoicedNoiseGain);
    noiseMix = float_to_q16(kCarrierNoiseMix);
    noiseShapeCoeff = float_to_q16(1.0f - expf(-2.0f * (float)M_PI * kNoiseShapeFreqHz / (float)kSampleRateHz));
}

void CarrierGenerator::begin_buffer(float carrierHz, uint8_t newWave, uint8_t oldWave) {
    wave = newWave;
    prevWave = oldWave;
    phaseInc = (uint32_t)((carrierHz * kTableSize / (float)kSampleRateHz) * 65536.0f);
    impulseHeight = float_to_q16(kVoicedLevel * (1.0f / sqrtf(3.0f)) *
                                 sqrtf((float)kSampleRateHz / carrierHz));
    float pitchScale = sqrtf(kWaveRefHz / carrierHz);
    sawLevel    = float_to_q16(kVoicedLevel * kSawLevelAtRef * pitchScale);
    squareLevel = float_to_q16(kVoicedLevel * kSquareLevelAtRef * pitchScale);
}