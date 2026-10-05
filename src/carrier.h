#pragma once
// =====================================================================
// carrier.h - Carrier-Erzeugung (stimmhaft + Rauschen)
// =====================================================================
// Wellenformen: Impulszug, Sägezahn, Rechteck, Rauschen (Auswahl siehe
// carrier_wave.h). Alle stimmhaften Carrier sind so abgeglichen, dass
// sie im Formantbereich dieselbe Leistung pro Band liefern wie das
// Rauschen der Zischlaute - dann passt EINE Bandgewichtung für alle.
// Herleitung und Messwerte: carrier.cpp und DEVLOG.
//
// Pro Puffer einmal begin_buffer() (Tonhöhe, Pegel, Wellenwechsel),
// pro Sample next() - inline, läuft im Sample-Takt auf Core0.

#include "fixed_point.h"
#include "carrier_wave.h"
#include "config.h"

struct CarrierGenerator {
    static constexpr int kTableSize = 512;

    q16 sawTable[kTableSize];
    uint32_t phase = 0;
    uint32_t noiseState = 12345;   // linearer Kongruenzgenerator
    q16 preEmphasis = 0;           // a in y = x - a*x[n-1]
    q16 sawPrev = 0;
    q16 squarePrev = 0;
    q16 unvoicedNoiseGain = 0;
    // Alter fester Rauschanteil im stimmhaften Carrier (kCarrierNoiseMix,
    // jetzt 0) - Pfad bleibt zum Vergleich erhalten.
    q16 noiseMix = 0;
    q16 noiseShapeCoeff = 0;
    q16 noiseShapeState = 0;

    // Pro Puffer gesetzt (begin_buffer):
    uint32_t phaseInc = 0;
    q16 impulseHeight = 0;
    q16 sawLevel = 0;
    q16 squareLevel = 0;
    uint8_t wave = kWaveImpulse;
    uint8_t prevWave = kWaveImpulse;

    void init();
    void begin_buffer(float carrierHz, uint8_t newWave, uint8_t oldWave);

    inline q16 next_noise() {
        noiseState = noiseState * 1664525u + 1013904223u;
        int32_t raw = (int32_t)(noiseState >> 8);          // obere 24 Bit
        return (raw & (kQ16One - 1)) - (kQ16One >> 1);      // grob [-0.5, 0.5)
    }

    // Ein Sample: stimmhafter Carrier und Rauschen (für stimmlose Laute).
    // i = Sample-Index im Puffer (für die Überblendung beim Wellenwechsel).
    inline void next(uint32_t i, q16 &voicedOut, q16 &unvoicedOut) {
        // Impulszug: ein Impuls bei jedem Phasenüberlauf, sonst 0.
        uint32_t idxNow  = (phase >> 16) & (kTableSize - 1);
        uint32_t idxNext = ((phase + phaseInc) >> 16) & (kTableSize - 1);
        q16 impulseQ16 = (idxNext < idxNow) ? impulseHeight : 0;
        q16 noiseQ16 = next_noise();
        q16 unvoicedQ16 = q16_mul(noiseQ16, unvoicedNoiseGain);

        // Sägezahn und Rechteck aus derselben Phase, jeweils mit
        // Höhenanhebung. Beide laufen immer mit, damit ihr Filterzustand
        // beim Umschalten stimmt.
        q16 sawRaw = sawTable[idxNow];
        q16 squareRaw = (idxNow < (uint32_t)(kTableSize / 2)) ? kQ16One : -kQ16One;
        q16 sawEmph = sawRaw - q16_mul(preEmphasis, sawPrev);
        q16 squareEmph = squareRaw - q16_mul(preEmphasis, squarePrev);
        sawPrev = sawRaw;
        squarePrev = squareRaw;

        q16 voicedQ16 = select(wave, impulseQ16, sawEmph, squareEmph, unvoicedQ16);
        if (wave != prevWave) {
            // Lineare Überblendung über diesen einen Puffer (~11.6 ms).
            q16 voicedOld = select(prevWave, impulseQ16, sawEmph, squareEmph, unvoicedQ16);
            q16 t = (q16)(((int64_t)i << kQ16Frac) / kBufferSamples);
            voicedQ16 = voicedOld + q16_mul(t, voicedQ16 - voicedOld);
        }
        noiseShapeState = noiseShapeState + q16_mul(noiseShapeCoeff, noiseQ16 - noiseShapeState);
        voicedQ16 = voicedQ16 + q16_mul(noiseMix, noiseShapeState - voicedQ16);

        phase += phaseInc;
        voicedOut = voicedQ16;
        unvoicedOut = unvoicedQ16;
    }

private:
    inline q16 select(uint8_t w, q16 impulse, q16 sawEmph, q16 squareEmph, q16 noiseWhite) const {
        switch (w) {
            case kWaveSaw:    return q16_mul(sawEmph, sawLevel);
            case kWaveSquare: return q16_mul(squareEmph, squareLevel);
            case kWaveNoise:  return noiseWhite;
            default:          return impulse;
        }
    }
};