#pragma once
// =====================================================================
// config.h - zentrale Konstanten des Vocoders
// =====================================================================
// Hardware-Zuordnung (Pins), Abtastung und Aufbau der Filterbank.
// Alles, was mehrere Module gemeinsam brauchen, steht hier. Feineinstel-
// lungen einzelner Module (Kompressor, Gate, Stimmhaft/Stimmlos, Carrier-
// Pegel) bleiben beim jeweiligen Modul.
//
// Herkunft und Begründung der Werte: siehe DEVLOG und git-Historie
// (Tag baseline-gut, Branch referenz-port, feature/carrier-waveforms).
//
// Hinweis: Die DAC-Pins (PCM5102A) stehen NICHT hier, sondern in der
// CMakeLists.txt (PICO_AUDIO_I2S_DATA_PIN=18, PICO_AUDIO_I2S_CLOCK_PIN_BASE=16
// -> BCK=GP16, LRCK=GP17, DIN=GP18), weil pico-extras sie dort erwartet.

#include <cmath>
#include <cstdint>
#include "pico/types.h"

// --- Pins ---------------------------------------------------------------

// Pots am internen ADC:
// GP26 (Pin 31): Formant-Verschiebung (Bändertausch, Hauptmerkmal).
// Die Carrier-Tonhöhe ist derzeit FEST (kMinCarrierHz, siehe unten). Ein
// eigenes Tonhöhen-Poti kann später an GP27 (Pin 32): kPitchPotEnabled =
// true setzen. GP28 ist frei.
// Schleifer an den GPIO, Außenbeine an 3V3(OUT) und ADC-GND (Pin 33).
constexpr uint    FORMANT_ADC_GPIO    = 26;
constexpr uint8_t FORMANT_ADC_CHANNEL = 0;
constexpr bool    kPitchPotEnabled    = false;
constexpr uint    POT_ADC_GPIO        = 27;   // Tonhöhe (nur wenn kPitchPotEnabled)
constexpr uint8_t POT_ADC_CHANNEL     = 1;

// INMP441 (I2S-Mic), Verkabelung siehe wiring-diagram.svg / README.md.
// SCK und WS MÜSSEN aufeinanderfolgende GPIOs sein (WS = SCK + 1), weil
// beide per side_set gesetzt werden. L/R des INMP441 auf GND.
// Bewusst getrennt von den DAC-Pins (GP16-18).
constexpr uint kMicSckPin = 10;
constexpr uint kMicWsPin  = 11;
constexpr uint kMicSdPin  = 12;

// Bedienung: Taster schaltet die Carrier-Wellenform weiter, die Onboard-
// LED zeigt sie als Blinkcode (1x Impulszug, 2x Sägezahn, 3x Rechteck,
// 4x Rauschen).
// Taster: GP14 (Pin 19) gegen GND (Pin 18), interner Pullup -> gedrückt = 0.
// 6x6-Taster am besten DIAGONAL anschließen (Einbaurichtung dann egal).
// LED: GP25 = Onboard-LED des Pico (NICHT beim Pico W - dort am WLAN-Chip).
constexpr uint kWaveButtonPin = 14;
constexpr uint kStatusLedPin  = 25;

// 7-Segment-Anzeige (TM1637, 4 Stellen mit Doppelpunkt), siehe display.h.
// VCC an 3V3(OUT) (Pin 36), NICHT 5 V (Pullups des Moduls an CLK/DIO).
constexpr uint    kDisplayClkPin     = 20;   // Pin 26
constexpr uint    kDisplayDioPin     = 21;   // Pin 27
constexpr uint8_t kDisplayBrightness = 2;    // 0..7 (niedrig = weniger Strom)

// --- Abtastung ------------------------------------------------------------

// 22.05 kHz: 10 Bänder mit je Analyse- und Synthesefilter passen damit
// (auf zwei Kerne verteilt) ins Budget; Nyquist 11 kHz liegt weit über
// dem höchsten Band (6 kHz).
constexpr uint32_t kSampleRateHz  = 22050;
constexpr uint32_t kBufferSamples = 256;   // = ~11.6 ms Budget pro Puffer

// --- Carrier-Tonhöhe ------------------------------------------------------

// VORÜBERGEHEND fest auf 110 Hz für reproduzierbare A/B-Tests und die
// Pegelkalibrierung der Wellenformen. Normalbereich: etwa 80-200 Hz
// (darüber werden die unteren Bänder dünn, siehe DEVLOG).
constexpr float kMinCarrierHz = 110.0f;
constexpr float kMaxCarrierHz = 110.0f;

// --- Filterbank -------------------------------------------------------------

// Nachbau der Software-Referenz (modular-synth, VocoderBands.ts):
// 10 Bänder, 90-6000 Hz geometrisch, Analyse-Q = 5.
// Abweichung: Synthese-Q = 2 statt 5 - bei Q=5 fällt pro Syntheseband
// meist nur EIN Oberton des Carriers (Orgelklang, kaum Vokalfarbe).
// Q~2 ist für 10 Bänder (~0.68 Oktaven Abstand) etwa lückenlos.
constexpr int   kNumBands       = 10;
constexpr float kBandFreqLowHz  = 90.0f;
constexpr float kBandFreqHighHz = 6000.0f;
constexpr float kBandQ          = 5.0f;
constexpr float kSynthesisQ     = 2.0f;

// Hüllkurve wie Tone.Follower(0.02): Gleichrichter + einpoliger Tiefpass
// mit 1/0.02 s = 50 Hz Grenzfrequenz -> Zeitkonstante 0.02/(2*pi) = 3.18 ms,
// symmetrisch (gleich schnell steigend wie fallend).
constexpr float kFollowerSmoothingS = 0.02f;
constexpr float kFollowerTauMs = 1000.0f * kFollowerSmoothingS / (2.0f * (float)M_PI);