// =====================================================================
// voiced_unvoiced.cpp - Parameter und Initialisierung
// =====================================================================
// Ausgelagert aus main.cpp, inhaltlich unverändert.

#include "voiced_unvoiced.h"
#include "config.h"
#include <cmath>

namespace {
// Rumpelfilter (Hochpass 80 Hz, Butterworth) VOR allem anderen: Bei
// leisen Lauten (sch, f) schwankte der Mic-Pegel tieffrequent stark
// (Atem/Luftstrom aufs Mic, micDc -0.09..-0.22) - das landete voll im
// 1.5-kHz-Tiefpass der Erkennung und überstimmte den leisen Hochton-
// anteil (sch: 0-16% stimmlos trotz Energie fast nur > 2 kHz). Im
// Browser entfernt der Mic-Eingang solches Rumpeln. Keine Sprach-
// information unter 80 Hz.
constexpr float kRumbleHighpassHz   = 80.0f;
constexpr float kButterworthQDb     = -3.0103f; // Q = 0.7071 linear, Web-Audio-Q in dB
constexpr float kTrebleBoostDb      = 6.0f;     // trebleBoost-Default der Referenz
constexpr float kTrebleShelfHz      = 2000.0f;
constexpr float kVuvSwitchHz        = 1500.0f;  // SWITCH_FREQUENCY
constexpr float kVuvFilterQDb       = 1.0f;     // Tone.Filter-Default (Web Audio: Q in dB)
constexpr float kVuvDetectorSmoothingS = 0.015f; // DETECTOR_SMOOTHING
constexpr float kVuvSwitchSmoothingS   = 0.01f;  // SWITCH_SMOOTHING
} // namespace

void VoicedUnvoiced::init() {
    const float fs = (float)kSampleRateHz;
    rumbleHighpass.setHighpass(kRumbleHighpassHz, kButterworthQDb, fs);
    trebleShelf.setHighShelf(kTrebleShelfHz, kTrebleBoostDb, fs);
    highpass.setHighpass(kVuvSwitchHz, kVuvFilterQDb, fs);
    lowpass.setLowpass(kVuvSwitchHz, kVuvFilterQDb, fs);
    // Tone.Follower(s) = einpoliger Tiefpass mit Grenzfrequenz 1/s,
    // also Zeitkonstante tau = s / (2*pi).
    float tauDet = kVuvDetectorSmoothingS / (2.0f * (float)M_PI);
    float tauSw  = kVuvSwitchSmoothingS / (2.0f * (float)M_PI);
    detectorCoeff = kQ16One - float_to_q16(expf(-1.0f / (tauDet * fs)));
    switchCoeff   = kQ16One - float_to_q16(expf(-1.0f / (tauSw * fs)));
    for (int k = 0; k <= kFadeTableSize; ++k) {
        float a = (float)k / (float)kFadeTableSize * (float)M_PI * 0.5f;
        fadeCos[k] = float_to_q16(cosf(a));
        fadeSin[k] = float_to_q16(sinf(a));
    }
}