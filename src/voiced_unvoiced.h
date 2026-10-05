#pragma once
// =====================================================================
// voiced_unvoiced.h - Rumpelfilter, Höhenanhebung, Stimmhaft/Stimmlos
// =====================================================================
// Nachbau von modular-synth VoicedUnvoicedNode.tsx (vgl. Doepfer A-129/5):
//   Mic -> Rumpelfilter (Hochpass 80 Hz) -> High-Shelf +6 dB @ 2 kHz
//   ("trebleBoost") = Modulator für die Analyse (Pre-Emphasis).
//   Erkennung: Hochpass- gegen Tiefpass-Pegel bei 1.5 kHz, je ein
//   Follower (tau 2.4 ms); Hochpass > Tiefpass -> stimmlos (hart 0/1),
//   geglättet (tau 1.6 ms). Carrier: gleichstarke Überblendung zwischen
//   stimmhaftem Carrier und Rauschen.
// Läuft pro Sample auf Core0 VOR den Bändern -> alles inline.
// Parameter und Begründungen: voiced_unvoiced.cpp.

#include "fixed_point.h"
#include "biquad_fixed.h"

struct VoicedUnvoiced {
    static constexpr int kFadeTableSize = 256;

    BiquadGeneralFixed rumbleHighpass;
    BiquadGeneralFixed trebleShelf;
    BiquadGeneralFixed highpass;      // Erkennung, 1.5 kHz
    BiquadGeneralFixed lowpass;       // Erkennung, 1.5 kHz
    q16 detectorCoeff = 0;            // 1 - exp(-1/(tau*fs))
    q16 switchCoeff = 0;
    q24 highEnv = 0;
    q24 lowEnv = 0;
    q16 fade = 0;                     // 0 = stimmhaft, 1 = stimmlos
    q16 fadeCos[kFadeTableSize + 1];  // gleichstarke Überblendung (Tone.CrossFade)
    q16 fadeSin[kFadeTableSize + 1];

    void init();

    // Ein Mic-Sample (Q16) verarbeiten. Liefert den vorverzerrten
    // Modulator (Q16) für die Analyse und aktualisiert die Überblendung.
    // isUnvoiced: harte Entscheidung dieses Samples (für die Diagnose).
    inline q16 process(q16 micQ16, bool &isUnvoiced) {
        q24 deRumbled = rumbleHighpass.processQ24(micQ16 << (kQ24Frac - kQ16Frac));
        q24 speech = trebleShelf.processQ24(deRumbled);

        q24 hp = highpass.processQ24(speech);
        q24 lp = lowpass.processQ24(speech);
        q24 hpAbs = (hp < 0) ? -hp : hp;
        q24 lpAbs = (lp < 0) ? -lp : lp;
        highEnv += (q24)(((int64_t)detectorCoeff * (hpAbs - highEnv)) >> kQ16Frac);
        lowEnv  += (q24)(((int64_t)detectorCoeff * (lpAbs - lowEnv)) >> kQ16Frac);
        q16 unvoiced = (highEnv > lowEnv) ? kQ16One : 0;  // GreaterThan(0)
        fade += q16_mul(switchCoeff, unvoiced - fade);
        isUnvoiced = (unvoiced != 0);

        return speech >> (kQ24Frac - kQ16Frac);
    }

    // Stimmhaften Carrier und Rauschen nach aktueller Überblendung mischen.
    inline q16 crossfade(q16 voicedQ16, q16 unvoicedQ16) const {
        int fadeIdx = (int)(((int64_t)fade * kFadeTableSize) >> kQ16Frac);
        if (fadeIdx < 0) fadeIdx = 0;
        if (fadeIdx > kFadeTableSize) fadeIdx = kFadeTableSize;
        return q16_mul(voicedQ16, fadeCos[fadeIdx]) + q16_mul(unvoicedQ16, fadeSin[fadeIdx]);
    }
};