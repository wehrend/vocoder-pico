// =====================================================================
// dynamics.cpp - Parameter und Initialisierung von Kompressor, Gate und
// Ausgangs-Tiefpass
// =====================================================================
// Ausgelagert aus main.cpp, inhaltlich unverändert.

#include "dynamics.h"
#include "config.h"

namespace {
// --- Kompressor ----------------------------------------------------------
// Gain-Struktur nach der Referenz (VocoderAnalysisNode.tsx /
// VocoderSynthNode.tsx): Hüllkurve x100 -> VCA pro Band -> Summe -> x10
// -> Tone.Compressor(-35 dB, 8:1, 5 ms / 150 ms) -> Makeup x6.
// Der x100-Boost steckt in der Bandgewichtung (main.cpp).
// Abweichung: Eingang x0.2 statt x10 - der Hardware-Pegel liegt ~30 dB
// über dem Browser (Reduktion war -55 dB statt ~-25 dB, siehe DEVLOG).
// Nicht nachgebaut: Soft-Knee und die automatische Makeup-Verstärkung
// des Web-Audio-DynamicsCompressor.
constexpr float kCompInputGain   = 0.4f;   // war 0.2 - mehr Kompression, leise Anteile (Konsonanten, Silbenenden) werden angehoben
constexpr float kCompThresholdDb = -35.0f;
constexpr float kCompRatio       = 8.0f;
constexpr float kCompAttackMs    = 5.0f;
constexpr float kCompReleaseMs   = 150.0f;
constexpr float kCompMakeup      = 6.0f;

// --- Noise-Gate ------------------------------------------------------------
// Die Referenz hat KEIN Gate. Hier trotzdem nötig: Mit der starken
// Kompression würde der Grundpegel in Sprechpausen hörbar angehoben.
// Detektor: geglättete Summe der Analyse-Hüllkurven (reines Mic-Signal,
// unabhängig von Carrier/Gain/Kompressor). Schwellen: dynamics.h.
constexpr float kGateDetAttackMs  = 5.0f;
constexpr float kGateDetReleaseMs = 50.0f;
constexpr float kGateAttackMs     = 5.0f;
constexpr float kGateReleaseMs    = 40.0f;

// --- Ausgangs-Tiefpass -----------------------------------------------------
// Aus der Talkbox-Phase (gegen scharfe Obertöne eines schmalen Pulses).
// Abgeschaltet (OutputLowpass::kEnabled = false), Pfad zum Vergleich
// erhalten.
constexpr float kOutputLowpassFreqHz = 3000.0f;

inline q16 time_coeff(float ms) {
    return float_to_q16(expf(-1.0f / (0.001f * ms * (float)kSampleRateHz)));
}
} // namespace

void Compressor::init() {
    inputGain = float_to_q16(kCompInputGain);
    thresholdLin = powf(10.0f, kCompThresholdDb / 20.0f);
    exponent = 1.0f / kCompRatio - 1.0f;
    attackCoeff = time_coeff(kCompAttackMs);
    releaseCoeff = time_coeff(kCompReleaseMs);
    makeup = float_to_q16(kCompMakeup);
    gain = kQ16One;
}

void NoiseGate::init() {
    detAttackCoeff = time_coeff(kGateDetAttackMs);
    detReleaseCoeff = time_coeff(kGateDetReleaseMs);
    openThreshold = float_to_q16(kGateOpenThreshold);
    closeThreshold = float_to_q16(kGateCloseThreshold);
    attackCoeff = time_coeff(kGateAttackMs);
    releaseCoeff = time_coeff(kGateReleaseMs);
}

void OutputLowpass::init() {
    coeff = float_to_q16(1.0f - expf(-2.0f * (float)M_PI * kOutputLowpassFreqHz / (float)kSampleRateHz));
}