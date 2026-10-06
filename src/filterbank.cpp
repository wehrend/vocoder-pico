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
// Bänder 0/1 (90/144 Hz) abgeschwächt: dort sitzt fast nur der Grundton
// der Stimme, der die Formanten verdeckt. Test 6.10.: leicht besser
// verständlich; stärkere Abschwächung {0.15, 0.3, 0.7, ...} war schlechter.
constexpr float kBandOutputGain[kNumBands] = {0.3f, 0.5f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
constexpr float kBandTiltRefHz = 365.0f;   // Band 3 behält Gewicht 1.0
// Hüllkurven-Verstärkung der Referenz (ANALYSIS_GAIN_BOOST, x100), in
// die Bandgewichtung eingerechnet (VCA-Verstärkung pro Band).
constexpr int kAnalysisGainBoost = 100;
q16 bandOutputGainQ16[kNumBands];

// Hüllkurven aller Bänder für den aktuellen Puffer (Phase 1 -> Phase 2),
// Q8.24. 10 x 256 x 4 Byte = 10 KB.
q24 envBlock[kNumBands][kBufferSamples];

// Formant-Zuordnung pro Syntheseband: Quellband und Überblendanteil (Q16).
// Standard = keine Verschiebung (Band b -> Band b).
// Initialisiert in filterbank_init() (Verschiebung 0: Band b -> Band b).
int formantSrc[kNumBands];
q16 formantFrac[kNumBands];

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
    filterbank_set_formant_shift(0.0f);
}

void filterbank_set_formant_shift(float shiftBands) {
    if (shiftBands >  kFormantMaxShiftBands) shiftBands =  kFormantMaxShiftBands;
    if (shiftBands < -kFormantMaxShiftBands) shiftBands = -kFormantMaxShiftBands;
    // Syntheseband b nimmt die Hüllkurve an Position p = b - shift
    // (Energie aus Analyseband a landet in Syntheseband a + shift).
    // Zwischen den beiden Nachbarbändern von p wird linear überblendet;
    // außerhalb des Bandbereichs ist die Hüllkurve 0.
    for (int b = 0; b < kNumBands; ++b) {
        float p = (float)b - shiftBands;
        int i0 = (int)floorf(p);
        float frac = p - (float)i0;
        q16 fracQ16 = float_to_q16(frac);
        if (fracQ16 >= kQ16One) { ++i0; fracQ16 = 0; }    // Rundung nahe 1.0
        formantSrc[b] = i0;
        formantFrac[b] = fracQ16;
    }
}

void filterbank_analyze(int firstBand, int endBand,
                        const q16 *modulator, q16 *gateDetOut) {
    for (uint32_t i = 0; i < kBufferSamples; ++i) {
        q16 micQ16 = modulator[i];
        q16 gateDet = 0;
        for (int b = firstBand; b < endBand; ++b) {
            bands[b].analyze(micQ16);
            envBlock[b][i] = bands[b].envelopeQ24;
            q16 env = bands[b].envelope;
            if (env > g_filterbankDiag.bandMax[b]) g_filterbankDiag.bandMax[b] = env;
            g_filterbankDiag.bandSum[b] += env;
            gateDet += env;
        }
        gateDetOut[i] = gateDet;
    }
}

// Hüllkurve (Q24) an Position (src + frac) zum Sample i; außerhalb 0.
static inline q24 shifted_envelope(int src, q16 frac, uint32_t i) {
    q24 e0 = (src >= 0 && src < kNumBands) ? envBlock[src][i] : 0;
    if (frac == 0) return e0;                       // ganze Bänder: exakt, ohne Rundung
    q24 e1 = (src + 1 >= 0 && src + 1 < kNumBands) ? envBlock[src + 1][i] : 0;
    return e0 + (q24)(((int64_t)(e1 - e0) * frac) >> kQ16Frac);
}

void filterbank_synthesize(int firstBand, int endBand,
                           const q16 *carrier, q16 *mixOut) {
    for (uint32_t i = 0; i < kBufferSamples; ++i) {
        q16 synthCarrierQ16 = carrier[i];
        q16 mix = 0;
        for (int b = firstBand; b < endBand; ++b) {
            if (kSoloBand < 0 || kSoloBand == b) {
                q24 envQ24 = shifted_envelope(formantSrc[b], formantFrac[b], i);
                // VCA: Hüllkurve x (Gewichtung x Referenz-Boost) zuerst,
                // dann mit dem gefilterten Carrier multiplizieren - so
                // bleibt die volle Q24-Auflösung der Hüllkurve erhalten.
                // Die Gewichtung gehört zum SYNTHESEband (gleicht den
                // Carrier aus), nicht zum Quellband.
                q16 vcaGain = (q16)(((int64_t)envQ24 * bandOutputGainQ16[b]) >> kQ24Frac);
                q16 filteredCarrier = bands[b].synthesisFilter.process(synthCarrierQ16);
                q16 bandOut = q16_mul(filteredCarrier, vcaGain);
                mix += bandOut;
                g_filterbankDiag.outSum[b] += (bandOut < 0) ? -bandOut : bandOut;
            }
        }
        mixOut[i] = mix;
    }
}