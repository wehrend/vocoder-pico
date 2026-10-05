#pragma once
// =====================================================================
// dynamics.h - Kompressor, Noise-Gate, (optionaler) Ausgangs-Tiefpass
// =====================================================================
// Läuft pro Sample auf Core0 nach dem Zusammenführen der Bänder ->
// alles inline. Parameter und Begründungen: dynamics.cpp.

#include <cmath>
#include "fixed_point.h"

// Gate-Schwellen hier (statt in dynamics.cpp), weil die Diagnose sie
// mit anzeigt. Detektor = geglättete Summe der Analyse-Hüllkurven.
// Stille nach Rumpelfilter: max. ~0.001; "fff": ~0.03 (siehe DEVLOG).
constexpr float kGateOpenThreshold  = 0.015f; // war 0.040
constexpr float kGateCloseThreshold = 0.008f; // war 0.020 - ~8x über den Stille-Spitzen

// --- Kompressor (rechnet in dB wie Tone.Compressor) ---------------------
struct Compressor {
    static constexpr uint32_t kGainUpdateSamples = 16;

    q16 inputGain = 0;
    float thresholdLin = 0.0f;
    float exponent = 0.0f;       // 1/ratio - 1
    q16 attackCoeff = 0;
    q16 releaseCoeff = 0;
    q16 makeup = 0;
    q16 envelope = 0;
    q16 gain = kQ16One;
    q16 minGainSeen = kQ16One;   // Diagnose: stärkste Reduktion im Fenster

    void init();

    // i = Sample-Index im Puffer: die Verstärkung wird alle
    // kGainUpdateSamples Samples per powf neu berechnet, dazwischen gehalten.
    inline q16 process(q16 mixed, uint32_t i) {
        mixed = q16_mul(mixed, inputGain);
        // Schutz gegen int32-Überlauf bei extremen Pegeln (Plosiv direkt
        // ins Mic) - weit über allem, was Sprache erreicht.
        constexpr q16 kMixedLimit = 16384 * kQ16One;
        if (mixed > kMixedLimit) mixed = kMixedLimit;
        if (mixed < -kMixedLimit) mixed = -kMixedLimit;
        q16 absMixed = (mixed < 0) ? -mixed : mixed;
        q16 coeff = (absMixed > envelope) ? attackCoeff : releaseCoeff;
        envelope = envelope + q16_mul(kQ16One - coeff, absMixed - envelope);
        // Über der Schwelle wird der Pegelüberschuss in dB durch die Ratio
        // geteilt: gain = (env / thr)^(1/ratio - 1)
        if ((i % kGainUpdateSamples) == 0) {
            float envLin = (float)envelope / (float)kQ16One;
            float g = 1.0f;
            if (envLin > thresholdLin) {
                g = powf(envLin / thresholdLin, exponent);
            }
            gain = float_to_q16(g);
            if (gain < minGainSeen) minGainSeen = gain;
        }
        mixed = q16_mul(mixed, gain);
        return q16_mul(mixed, makeup);
    }
};

// --- Noise-Gate mit Hysterese (Schmitt-Trigger-Muster) ------------------
struct NoiseGate {
    q16 detAttackCoeff = 0;
    q16 detReleaseCoeff = 0;
    q16 detector = 0;
    q16 openThreshold = 0;
    q16 closeThreshold = 0;
    q16 attackCoeff = 0;
    q16 releaseCoeff = 0;
    q16 gain = 0;
    bool isOpen = false;

    void init();

    // Detektor nachführen (Rohwert = Summe der Analyse-Hüllkurven).
    inline q16 update_detector(q16 raw) {
        q16 coeff = (raw > detector) ? detAttackCoeff : detReleaseCoeff;
        detector = detector + q16_mul(kQ16One - coeff, raw - detector);
        return detector;
    }

    inline q16 apply(q16 x) {
        q16 threshold = isOpen ? closeThreshold : openThreshold;
        bool shouldBeOpen = (detector > threshold);
        isOpen = shouldBeOpen;
        q16 target = shouldBeOpen ? kQ16One : 0;
        q16 coeff = (target > gain) ? attackCoeff : releaseCoeff;
        gain = gain + q16_mul(kQ16One - coeff, target - gain);
        return q16_mul(x, gain);
    }
};

// --- Ausgangs-Tiefpass (derzeit abgeschaltet, siehe dynamics.cpp) -------
struct OutputLowpass {
    static constexpr bool kEnabled = false;
    q16 coeff = 0;
    q16 state = 0;

    void init();

    inline q16 process(q16 x) {
        if (!kEnabled) return x;
        state = state + q16_mul(coeff, x - state);
        return state;
    }
};