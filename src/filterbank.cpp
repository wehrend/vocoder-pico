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
// Zwei getrennte Gewichtungen (wichtig für die Formant-Verschiebung):
//
// kSourceWeight - gehört zum ANALYSEband (Quelle): Bänder 0/1 (90/144 Hz)
// tragen fast nur den Grundton der Stimme, der die Formanten verdeckt.
// Test 6.10.: {0.3, 0.5, ...} leicht besser verständlich, stärkere
// Abschwächung {0.15, 0.3, 0.7, ...} schlechter. Weil das eine Eigenschaft
// der QUELLE ist, wandert die Abschwächung bei der Formant-Verschiebung
// mit (vorher hing sie am Syntheseband - der Grundton landete beim
// Verschieben ungedämpft in den Bändern 2-4, siehe DEVLOG).
//
// kBandOutputGain - gehört zum SYNTHESEband: manueller Feinabgleich (1.0),
// zusätzlich zur Carrier-Gewichtung sqrt(kBandTiltRefHz / f).
constexpr float kSourceWeight[kNumBands]   = {0.3f, 0.5f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
constexpr float kBandOutputGain[kNumBands] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
constexpr float kBandTiltRefHz = 365.0f;   // Band 3 behält Gewicht 1.0
// Hüllkurven-Verstärkung der Referenz (ANALYSIS_GAIN_BOOST, x100), in
// die Bandgewichtung eingerechnet (VCA-Verstärkung pro Band).
constexpr int kAnalysisGainBoost = 100;
float synthGain[kNumBands];   // Synthese-Gewichtung: kBandOutputGain * sqrt(365/f) * 100

// Hüllkurven aller Bänder für den aktuellen Puffer (Phase 1 -> Phase 2),
// Q8.24. Oben und unten je kEnvPad Nullzeilen: verschobene Zugriffe
// außerhalb des Bandbereichs lesen dort 0, ohne Bereichsprüfung im
// Sample-Takt. (10 + 8) x 256 x 4 Byte = 18 KB.
constexpr int kEnvPad = (int)kFormantMaxShiftBands + 1;
q24 envBlock[kNumBands + 2 * kEnvPad][kBufferSamples];

// Quellgewicht in Q8 (256 = 1.0). Wird in der Analyse-Phase direkt auf
// die gespeicherte Hüllkurve angewandt - nur für Bänder mit Gewicht != 1
// (eine 32-Bit-Multiplikation, auf dem M0+ ein Takt).
int32_t sourceWeightQ8[kNumBands];
// Synthese-Gewichtung pro Band in Q16 (für die eine 64-Bit-Multiplikation
// im VCA, wie vor der Formant-Verschiebung).
q16 synthGainQ16[kNumBands];

// Formant-Zuordnung pro Syntheseband, pro Puffer vorberechnet: Zeile der
// unteren Quelle im envBlock und Überblendanteil zur oberen in Q8
// (0..255). Überblendung per 32-Bit-Multiplikation:
//   env = e0 + ((e1 - e0) >> 8) * frac8
// statt zweier 64-Bit-Multiplikationen - die kosteten bei Verschiebungen
// zwischen zwei Bändern ~1 ms pro Puffer (DEVLOG).
int formantRow[kNumBands];
int32_t formantFrac8[kNumBands];

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
        synthGain[b] = kBandOutputGain[b] * tilt * (float)kAnalysisGainBoost;
        synthGainQ16[b] = float_to_q16(synthGain[b]);
        sourceWeightQ8[b] = (int32_t)lroundf(kSourceWeight[b] * 256.0f);
    }
    filterbank_set_formant_shift(0.0f);
}

void filterbank_set_formant_shift(float shiftBands) {
    if (shiftBands >  kFormantMaxShiftBands) shiftBands =  kFormantMaxShiftBands;
    if (shiftBands < -kFormantMaxShiftBands) shiftBands = -kFormantMaxShiftBands;
    // Syntheseband b nimmt die Hüllkurve an Position p = b - shift
    // (Energie aus Analyseband a landet in Syntheseband a + shift).
    // Zwischen den beiden Nachbarbändern von p wird linear überblendet;
    // außerhalb des Bandbereichs ist die Hüllkurve 0 (Nullzeilen).
    for (int b = 0; b < kNumBands; ++b) {
        float p = (float)b - shiftBands;
        int i0 = (int)floorf(p);
        int32_t frac8 = (int32_t)lroundf((p - (float)i0) * 256.0f);
        if (frac8 >= 256) { ++i0; frac8 = 0; }      // Rundung nahe 1.0
        formantRow[b] = i0 + kEnvPad;
        formantFrac8[b] = frac8;
    }
}

void filterbank_analyze(int firstBand, int endBand,
                        const q16 *modulator, q16 *gateDetOut) {
    for (uint32_t i = 0; i < kBufferSamples; ++i) {
        q16 micQ16 = modulator[i];
        q16 gateDet = 0;
        for (int b = firstBand; b < endBand; ++b) {
            bands[b].analyze(micQ16);
            q24 e = bands[b].envelopeQ24;
            int32_t w = sourceWeightQ8[b];
            // Hüllkurven sind >= 0, (e >> 8) * w passt bequem in 32 Bit.
            envBlock[b + kEnvPad][i] = (w == 256) ? e : (e >> 8) * w;
            q16 env = bands[b].envelope;
            if (env > g_filterbankDiag.bandMax[b]) g_filterbankDiag.bandMax[b] = env;
            g_filterbankDiag.bandSum[b] += env;
            gateDet += env;
        }
        gateDetOut[i] = gateDet;
    }
}

void filterbank_synthesize(int firstBand, int endBand,
                           const q16 *carrier, q16 *mixOut) {
    for (uint32_t i = 0; i < kBufferSamples; ++i) {
        q16 synthCarrierQ16 = carrier[i];
        q16 mix = 0;
        for (int b = firstBand; b < endBand; ++b) {
            if (kSoloBand < 0 || kSoloBand == b) {
                // VCA: Hüllkurve des Quellbands (ggf. zwischen zwei Bändern
                // überblendet) x Synthese-Gewichtung zuerst, dann mit dem
                // gefilterten Carrier - volle Q24-Auflösung der Hüllkurve.
                const int row = formantRow[b];
                q24 env = envBlock[row][i];
                const int32_t frac8 = formantFrac8[b];
                if (frac8 != 0) env += ((envBlock[row + 1][i] - env) >> 8) * frac8;
                q16 vcaGain = (q16)(((int64_t)env * synthGainQ16[b]) >> kQ24Frac);
                q16 filteredCarrier = bands[b].synthesisFilter.process(synthCarrierQ16);
                q16 bandOut = q16_mul(filteredCarrier, vcaGain);
                mix += bandOut;
                g_filterbankDiag.outSum[b] += (bandOut < 0) ? -bandOut : bandOut;
            }
        }
        mixOut[i] = mix;
    }
}