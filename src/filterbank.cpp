// =====================================================================
// filterbank.cpp - Bänder, Bandgewichtung, Verarbeitung pro Puffer
// =====================================================================
// Ausgelagert aus main.cpp, inhaltlich unverändert. Die Verarbeitung pro
// Sample (analyze, Synthesefilter) ist inline aus vocoder_band_fixed.h und
// biquad_fixed.h und wird hier in die Schleife eingebettet.

#include "filterbank.h"
#include "vocoder_band_fixed.h"
#include <cmath>

FilterbankDiag g_filterbankDiag;

namespace {
VocoderBandFixed bands[kNumBands];

// Bandgewichtung. Der Carrier (Impulszug/Sägezahn/Rechteck stimmhaft,
// weißes Rauschen stimmlos - alle auf gleiche Leistung pro Hz
// abgeglichen, siehe carrier.cpp) hat in Constant-Q-Bändern +3 dB/Oktave.
// Ausgleich analytisch: Gewicht = sqrt(kBandTiltRefHz / f). Simulation:
// alle Bänder dann bei ~0.09, für Impulse UND Rauschen gleich.
// kBandOutputGain bleibt als manueller Feinabgleich pro Band (1.0).
constexpr float kBandOutputGain[kNumBands] = {0.3f, 0.5f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
constexpr float kBandTiltRefHz = 365.0f;   // Band 3 behält Gewicht 1.0
// Hüllkurven-Verstärkung der Referenz (ANALYSIS_GAIN_BOOST, x100), in
// die Bandgewichtung eingerechnet (VCA-Verstärkung pro Band).
constexpr int kAnalysisGainBoost = 100;
q16 bandOutputGainQ16[kNumBands];

// DIAGNOSE-SCHALTER: -1 = normaler Mix aller Bänder. 0..kNumBands-1 =
// nur dieses eine Band hörbar (Analyse läuft für alle weiter).
constexpr int kSoloBand = -1;

inline float band_freq(int b) {
    // Geometrische Verteilung exakt wie vocoderBandFrequencies() in
    // VocoderBands.ts: t = b/(N-1), f = f_min * (f_max/f_min)^t.
    float t = (kNumBands == 1) ? 0.0f : (float)b / (float)(kNumBands - 1);
    return kBandFreqLowHz * powf(kBandFreqHighHz / kBandFreqLowHz, t);
}
} // namespace

void filterbank_init() {
    for (int b = 0; b < kNumBands; ++b) {
        float freq = band_freq(b);
        // Symmetrische Hüllkurve wie Tone.Follower: Attack = Release.
        bands[b].init(freq, kBandQ, kFollowerTauMs, kFollowerTauMs, (float)kSampleRateHz);
        // Synthesefilter mit eigenem Q (siehe kSynthesisQ in config.h).
        bands[b].synthesisFilter.setBandpass(freq, kSynthesisQ, (float)kSampleRateHz);
        float tilt = sqrtf(kBandTiltRefHz / freq);
        bandOutputGainQ16[b] = float_to_q16(kBandOutputGain[b] * tilt * (float)kAnalysisGainBoost);
    }
}

void filterbank_process(int firstBand, int endBand,
                        const q16 *modulator, const q16 *carrier,
                        q16 *mixOut, q16 *gateDetOut) {
    for (uint32_t i = 0; i < kBufferSamples; ++i) {
        q16 micQ16 = modulator[i];
        q16 synthCarrierQ16 = carrier[i];
        q16 mix = 0;
        q16 gateDet = 0;
        for (int b = firstBand; b < endBand; ++b) {
            bands[b].analyze(micQ16);
            if (kSoloBand < 0 || kSoloBand == b) {
                // VCA: Hüllkurve x (Gewichtung x Referenz-Boost) zuerst,
                // dann mit dem gefilterten Carrier multiplizieren - so
                // bleibt die volle Q24-Auflösung der Hüllkurve erhalten.
                q16 vcaGain = (q16)(((int64_t)bands[b].envelopeQ24 * bandOutputGainQ16[b]) >> kQ24Frac);
                q16 filteredCarrier = bands[b].synthesisFilter.process(synthCarrierQ16);
                q16 bandOut = q16_mul(filteredCarrier, vcaGain);
                mix += bandOut;
                g_filterbankDiag.outSum[b] += (bandOut < 0) ? -bandOut : bandOut;
            }
            q16 env = bands[b].envelope;
            if (env > g_filterbankDiag.bandMax[b]) g_filterbankDiag.bandMax[b] = env;
            g_filterbankDiag.bandSum[b] += env;
            gateDet += env;
        }
        mixOut[i] = mix;
        gateDetOut[i] = gateDet;
    }
}