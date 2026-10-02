// Talkbox-artiger Envelope-Follower auf generischem Raspberry Pi Pico +
// PCM5102A-Breakout - bewusst vereinfachtes Zwischenziel, nachdem die
// volle 12-Band-Filterbank (siehe DEVLOG bis Nachtrag 26) sich als zu
// große Baustelle auf einmal herausgestellt hat.
//
// PRINZIP, viel simpler als der Filterbank-Vocoder: EIN Bandpass +
// Gleichrichter + Attack/Release liefert EINE Gesamt-Hüllkurve aus dem
// Mic-Signal. Diese Hüllkurve moduliert direkt die Lautstärke eines
// Carrier-Oszillators (Pulszug, Tonhöhe per Poti) - reine Amplituden-
// modulation, KEINE spektrale Formung pro Frequenzband mehr. Kein
// Vocoder-Effekt (keine Formanten, keine erkennbaren Wörter), aber ein
// klar hörbares, reaktives Ergebnis: der Ton wird lauter/leiser im
// Rhythmus der Stimme - ein Talkbox-/Theremin-artiges Instrument.
//
// WARUM DAS EINE GUTE ZWISCHENSTATION IST: Es umgeht genau das
// Problem, an dem die Filterbank-Serie zuletzt hing (begrenzte
// Bandbreite des Mic-Vorverstärkers bei hohem Gain, siehe DEVLOG
// Nachtrag 23) - für eine reine Lautstärke-Hüllkurve ist es fast
// egal, ob hohe Frequenzen im Mic-Signal vollständig ankommen,
// Amplitude ist unabhängig vom Spektrum. Außerdem: die komplette
// Infrastruktur (Oszillator, Poti-Tonhöhe, DC-selbstkalibrierender
// Mic-Eingang, Kompressor, Noise-Gate) ist aus dem Vocoder-Projekt
// schon vorhanden und einzeln bewiesen funktionierend.
//
// GESTRICHEN gegenüber der ursprünglichen 12-Band-Vocoder-Version
// (nicht mehr gebraucht) - Stand nach dem Talkbox-Umbau, TEILWEISE
// durch den 3-Band-Ausbau (Nachtrag 44) wieder relativiert, siehe dort:
// - 12-Band-Filterbank -> jetzt wieder 3 Bänder (VocoderBandFixed),
//   aber bewusst nicht gleich wieder 12
// - Zwei-Kern-Aufteilung (Core1/pico_multicore) - Rechenlast bei 3
//   Bändern weiterhin trivial genug für einen einzelnen Kern
// - Stimmhaft/Unstimmhaft-Erkennung + Rauschmischung im Carrier - war
//   nur nötig, um unstimmhaften Lauten in der SPEKTRALEN Vocoder-
//   Verarbeitung Energie zu geben; für den 3-Band-Zwischenschritt noch
//   nicht wieder eingeführt (siehe Nachtrag 44)
// - Pre-Emphasis-Filter (Nachtrag 24-26) - war ein Versuch, den
//   analogen Bandbreiten-Verlust für die Vocoder-Formant-Erkennung
//   auszugleichen; bisher nicht wieder eingeführt
//
// BEHALTEN: Oszillator/Poti-Tonhöhe, DC-Tracking im Mic-Read,
// Kompressor, Noise-Gate (mit Hysterese), FreeRTOS-Grundgerüst
// (audioTask/controlTask, Priority-Starvation-Fix) - alles bereits
// einzeln verifiziert funktionierend, siehe DEVLOG.
//
// ABNAHMEKRITERIUM für das Talkbox-Zwischenziel (siehe DEVLOG): Ton
// wird beim Sprechen/Singen vor dem Mic hörbar lauter/leiser, bleibt
// bei Stille sauber still (Gate), verzerrt nicht bei normaler
// Sprechlautstärke (Kompressor). Erreicht und stabil (siehe Nachtrag
// 40), zunächst im eingegrenzten 180-300Hz-Bereich (Nachtrag 38/39),
// nach der Sternerdung (Nachtrag 47) über den vollen 80-400Hz-Bereich
// bestätigt stabil - siehe dort für die eigentliche Ursache
// (Masseschleife) und ihre Behebung.
//
// === NACHTRAG 44: AUSBAU AUF 3 BÄNDER (feature/multiband-vocoder) ===
// Erster schrittweiser Ausbau Richtung Mehrband-Vocoder, bewusst NICHT
// gleich auf 12 Bänder gesprungen (Lehre aus Nachtrag 27/34). Statt
// EINEM Bandpass + EINER Hüllkurve: 3 Bänder, jeweils mit eigenem
// Analyse-Bandpass (Mic) UND eigenem Synthese-Bandpass (Carrier) -
// das ist der eigentliche Unterschied zwischen Talkbox (reine
// Amplitudenmodulation) und Vocoder (spektrale Formung). Wiederverwendet
// `vocoder_band_fixed.h` unverändert aus dem 12-Band-Projekt. Noch
// KEINE Stimmhaft/Unstimmhaft-Erkennung, noch kein Ausbau der
// Bandzahl über 3 hinaus - erst dieser Schritt stabilisieren, dann
// nach demselben Muster weiter hochskalieren.

// === UMSTIEG AUF DAS INMP441 (digitales I2S-MEMS-Mikrofon) ===
// Der analoge Mic-Pfad (MAX4466 -> RP2040-ADC) ist komplett ersetzt.
// Zwei Gründe, siehe DEVLOG:
// 1. Der ADC wurde bisher in der Sample-Schleife per adc_read() gelesen,
//    also so schnell wie die Schleife lief (256/avg) statt mit
//    kSampleRateHz, und in Bursts mit Lücken bis zum nächsten I2S-Puffer.
//    Dadurch lagen alle Analysebänder um den Faktor budget/avg höher als
//    berechnet, und jede Pufferlücke erzeugte einen Sprung im Signal.
//    -> Alle bisherigen Kalibrierungen (Bänder, Gewichtung, Gate,
//    Kompressor, Mic-Bandbreitenmessung) sind damit NEU zu machen.
// 2. Bandbreite/Störanfälligkeit des analogen Vorverstärkers.
//
// Neuer Pfad: PIO-I2S-Empfänger (i2s_mic.pio, auf pio1) -> DMA in einen
// Ringpuffer, hardware-getaktet mit exakt kSampleRateHz. Die Audio-
// Schleife holt pro I2S-Ausgabepuffer einen Block von kBufferSamples
// Mic-Samples aus dem Ring (mic_read_block). Getestet vorab isoliert
// im Projekt hello_mic_test.
//
// Der Pot bleibt am internen ADC (GP26) - nicht zeitkritisch.

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "pico/stdlib.h"
#include "pico/audio_i2s.h"
#include "pico/time.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "i2s_mic.pio.h"
#include "fixed_point.h"
#include "biquad_fixed.h"
#include "vocoder_band_fixed.h"
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {

// --- Pin-Zuordnung ---
// Pot weiterhin am internen ADC. GP27 (früher Mic-ADC) ist jetzt frei.
constexpr uint POT_ADC_GPIO       = 26;
constexpr uint8_t POT_ADC_CHANNEL = 0;

// INMP441 (I2S-Mic), siehe hello_mic_test/README.md für die Verkabelung.
// SCK und WS MÜSSEN aufeinanderfolgende GPIOs sein (WS = SCK + 1),
// weil beide per side_set gesetzt werden. L/R des INMP441 auf GND.
// Bewusst getrennt von den DAC-Pins (GP16-18).
constexpr uint kMicSckPin = 10;
constexpr uint kMicWsPin  = 11;
constexpr uint kMicSdPin  = 12;

// Carrier-Tonhöhenbereich - unverändert.
// ZURÜCKGESETZT auf 80Hz (siehe DEVLOG Nachtrag 31): die Einengung auf
// 150/200Hz war eine Reaktion auf Kontamination, die zumindest teilweise
// durch die (jetzt ebenfalls zurückgesetzte) 400Hz-Envelope-Analyse
// mitverursacht wurde, nicht nur durch die Carrier-Frequenz selbst.
// 80Hz war der zuletzt bestätigt funktionierende Wert, direkt nach der
// DIN-Leitungs-Umverlegung.
// Auf die bestätigt sichere Mitte eingegrenzt (war 80-400Hz, dann
// 180-380Hz) - siehe DEVLOG Nachtrag 38/39: 380Hz war immer noch zu
// nah am oberen Problembereich (355-379Hz zeigte anhaltendes
// Hängenbleiben, teils über mehrere Sekunden und mehrere Poti-
// Stellungen hinweg). 184Hz und 262-266Hz bestätigt sauber. Die
// eigentliche elektrische Ursache ist weiterhin nicht gefunden, das
// hier bleibt ein pragmatisches Eingrenzen, keine Lösung.
// Voller Bereich wiederhergestellt (siehe DEVLOG Nachtrag 47): die
// Sternerdung hat die eigentliche elektrische Ursache (Masseschleife
// über den gemeinsamen Steckbrett-GND-Pfad, siehe Nachtrag 42/43)
// tatsächlich behoben, bestätigt über den kompletten 80-400Hz-Bereich.
// Die vorherige Eingrenzung auf 180-300Hz (Nachtrag 38/39) war nur ein
// Software-seitiges Umgehen des Symptoms, keine Lösung - jetzt nicht
// mehr nötig.
constexpr float kMinCarrierHz = 80.0f;
constexpr float kMaxCarrierHz = 400.0f;

// Rechenlast ist jetzt trivial (1 Bandpass statt 12) - volle 44.1kHz
// sind wieder problemlos drin, kein Grund mehr für die 22.05kHz-
// Absenkung aus der Vocoder-Serie.
// Von 44.1kHz auf 22.05kHz gesenkt (siehe DEVLOG Nachtrag 50) - 6
// Bänder (je Analyse+Synthese) überschreiten bei voller Samplerate das
// CPU-Budget (avg=6400-6700us gegen budget=5804us) - exakt dasselbe
// Muster, das beim ursprünglichen 12-Band-Projekt zur selben Maßnahme
// führte. Höchstes Band liegt bei 2000Hz, Nyquist bei 22.05kHz bleibt
// bei 11kHz - kein Informationsverlust.
constexpr uint32_t kSampleRateHz  = 22050;
constexpr uint32_t kBufferSamples = 256;

// --- 3-Band-Filterbank (siehe Nachtrag 44) ---
// Frequenzbereich bewusst innerhalb dessen gewählt, wo laut Vocoder-
// Diagnose (Nachtrag 23) noch brauchbar viel Mic-Signalenergie ankommt
// (Gain-Bandbreite-Kompromiss am MAX4466 lässt oberhalb von grob
// 1-1.5kHz kaum noch etwas durch). Geometrisch gestaffelt: 150Hz /
// ~424Hz / 1200Hz. Erster Schätzwert, kein gemessenes Optimum.
constexpr int kNumBands = 6;
// Exakt nach der bestätigt funktionierenden Software-Referenz
// (vocoderBands.ts, siehe DEVLOG Nachtrag 60): FREQ_MIN=90Hz,
// FREQ_MAX=6000Hz, Q=5, logarithmisch/geometrisch verteilt - deutlich
// verlässlicher als die zuvor übernommenen MFOS-Werte, weil das die
// eigene, bereits bewiesen funktionierende Implementierung ist statt
// eines fremden Projekts. Bandzahl bleibt bei 6 (nicht auf die vollen
// 10 der Referenz erhöht - CPU-Budget, siehe Nachtrag 50).
//
// WICHTIGER, bewusster Unterschied zur Referenz: Deren Kommentar
// verlangt explizit GLEICHES Q für Analyse UND Synthese - wir nutzen
// hier weiterhin getrenntes Synthese-Q (siehe kSynthesisQ, Nachtrag
// 54), weil unser einfacher Hardware-Pulston (anders als der
// vermutlich vollere Software-Carrier) bei gleichem Q=5 auf der
// Synthese-Seite ganze Bänder komplett stumm ließ (diskrete Obertöne
// trafen das schmale Fenster nicht). Kein Widerspruch zur Referenz an
// sich, sondern eine bewusste Anpassung an eine andere Carrier-
// Charakteristik - im Hinterkopf behalten, falls die Formanten
// dadurch "verschmiert" wirken (die Referenz warnt genau davor).
constexpr float kBandFreqLowHz  = 90.0f; // wie Referenz - unterer Bereich funktioniert einwandfrei
constexpr float kBandFreqHighHz = 6000.0f; // von 6000Hz zurückgesetzt (siehe DEVLOG Nachtrag 61) - Referenz-Wert ignoriert unsere analoge Mic-Bandbreitengrenze; Bänder 3-5 blieben bei 6000Hz-Obergrenze für JEDEN Vokal nahe 0 (totes Gewicht). 1500Hz ist der empirisch bestätigte, noch nutzbare Bereich nach dem 12k/12k-Teiler-Umbau.
constexpr float kBandQ          = 5.0f; // Analyse-Q - schmal für gute Formant-Trennung, identisch zur Referenz
// Synthese-Q bewusst SEPARAT und viel breiter als das Analyse-Q
// (siehe DEVLOG Nachtrag 54): bei Q=5 trifft der Carrier (bzw. seine
// diskreten Obertöne) das schmale Synthese-Fenster eines Bands nur
// zufällig - je nach exakter Tonhöhe bekommt ein Band mal viel, mal
// fast keine Carrier-Energie zum Formen, UNABHÄNGIG vom Sprachinhalt.
// Ein breiteres Synthese-Fenster garantiert verlässlich Energie in
// jedem Band, während die Analyse weiterhin scharf trennt.
constexpr float kSynthesisQ     = 1.2f;
// Tiefe Bänder etwas träger (Formanten bewegen sich langsamer), hohe
// Bänder etwas flinker (Konsonanten/Transienten) - dieselbe Logik wie
// im 12-Band-Projekt, siehe vocoder_band_fixed.h.
constexpr float kAttackMsLow    = 8.0f;
constexpr float kAttackMsHigh   = 3.0f;
constexpr float kReleaseMsLow   = 120.0f;
constexpr float kReleaseMsHigh  = 60.0f;

VocoderBandFixed bands[kNumBands];

// Ausgangsseitige Gewichtung pro Band, NACH der Synthese, VOR der
// Summierung - kompensiert den natürlichen Spektralabfall menschlicher
// Sprache (Stimmquelle fällt mit steigender Frequenz stark ab,
// unabhängig vom Vokal). Ohne das dominiert Band 0 die Summe um
// Faktor ~5 gegenüber dem höchsten Band, bei JEDEM Vokal gleichermaßen
// (siehe DEVLOG Nachtrag 51) - dadurch gehen die eigentlich
// vorhandenen, vokalabhängigen RELATIVEN Verschiebungen zwischen den
// Bändern im Summensignal akustisch unter. Werte grob am beobachteten
// ~5x-Gefälle über 6 Bänder kalibriert - erster Schätzwert, kein
// gemessenes Optimum.
// TESTWEISE neutralisiert (war {1.0, 1.4, 2.0, 2.7, 3.7, 5.0}), siehe
// DEVLOG Nachtrag 56: kalibriert für die ANALYSE-Seite (Sprache hat
// natürlich mehr tieffrequente Energie), passt aber vermutlich nicht
// zur SYNTHESE-Seite jetzt mit Rauschmix (recht gleichmäßige Energie
// übers Spektrum) - erst den Rauschmix isoliert prüfen, dann ggf.
// neu gewichten.
constexpr float kBandOutputGain[kNumBands] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}; // zurückgesetzt - die Gewichtung hat den einzigen bisher gehörten iii/uuu-Unterschied wieder verwischt, siehe DEVLOG Nachtrag 58
q16 g_bandOutputGainQ16[kNumBands];

// DIAGNOSE-SCHALTER: -1 = normaler Mix aller Bänder. 0..kNumBands-1 =
// nur dieses eine Band hörbar, alle anderen stumm - testet, ob die
// SYNTHESE-Seite selbst (Carrier gefiltert durch verschiedene Bänder)
// überhaupt hörbar unterschiedlich klingt, unabhängig vom Mic-Signal/
// Vokal. Siehe DEVLOG Nachtrag 53.
constexpr int kSoloBand = -1;

// Pulszug-Wavetable für den Carrier (siehe DEVLOG Nachtrag 13 für die
// Begründung: gleichmäßigeres Obertonspektrum als ein Sägezahn). Hier
// nur noch fürs Timbre des Tons selbst relevant, nicht mehr für
// Formant-Abdeckung wie beim Vocoder.
constexpr int kCarrierTableSize = 512;
q16 carrierTable[kCarrierTableSize];

// Einfacher Pseudozufalls-Rauschgenerator (linearer Kongruenzgenerator)
// für die Rauschbeimischung in den Carrier - siehe DEVLOG Nachtrag 55.
uint32_t g_noiseState = 12345;
inline q16 next_noise_q16() {
    g_noiseState = g_noiseState * 1664525u + 1013904223u;
    int32_t raw = (int32_t)(g_noiseState >> 8); // obere 24 Bit, gleichmäßiger verteilt
    return (raw & (kQ16One - 1)) - (kQ16One >> 1); // grob auf [-0.5, 0.5) skaliert
}

// Fester Rauschanteil im Carrier - siehe DEVLOG Nachtrag 55: ein rein
// periodischer Carrier hat nur diskrete, lückenhafte Obertöne, die je
// nach Tonhöhe zufällig ein Synthese-Fenster treffen oder verfehlen,
// UNABHÄNGIG vom Sprachinhalt. Rauschen liefert kontinuierliche,
// lückenlose Energie über das ganze Spektrum - jedes Band bekommt
// garantiert echte, unterscheidbare Substanz zum Formen. Noch KEINE
// Stimmhaft/Unstimmhaft-Umschaltung (das wäre der nächste, feinere
// Schritt) - erstmal ein fester Mix, um die Grundidee zu testen.
constexpr float kCarrierNoiseMix = 0.4f;
q16 g_carrierNoiseMixQ16 = 0;

// "Rosa" statt weißes Rauschen - siehe DEVLOG Nachtrag 56: weißes
// Rauschen hat gleiche Energie pro Hertz, aber unsere Bänder haben
// alle dasselbe Q -> höhere Bänder haben eine breitere absolute
// Bandbreite und fangen dadurch systematisch mehr vom weißen Rauschen
// ein, unabhängig von der Stimme (Band 5 wurde dadurch viel lauter
// als Band 0). Ein einfacher Tiefpass vor der Beimischung gibt dem
// Rauschen mehr Energie pro Oktave statt pro Hertz - passt zur
// konstanten-Q-Bandaufteilung. Grenzfrequenz grob in der Mitte des
// Bandbereichs, erster Schätzwert.
constexpr float kNoiseShapeFreqHz = 400.0f;
q16 g_noiseShapeCoeff = 0;
q16 g_noiseShapeState = 0;

void build_carrier_table() {
    // 50% statt der ursprünglichen 20% (die waren fürs breitbandige
    // Obertonspektrum des 12-Band-Vocoders gedacht, siehe Nachtrag 13 -
    // für die Talkbox nicht mehr nötig). Ein Rechtecksignal hat von
    // Natur aus weniger hochfrequente Energie als ein schmaler
    // Pulszug - löst das "unangenehm schrill bei hohen Tonhöhen"-
    // Problem an der Quelle, statt nachträglich zu filtern (siehe
    // DEVLOG Nachtrag 36 - der Ausgangs-Tiefpass hatte einen
    // unerklärten Nebeneffekt auf die Gate-Rückkopplung).
    constexpr float kDutyCycle = 0.5f;
    for (int i = 0; i < kCarrierTableSize; ++i) {
        float phase = (float)i / (float)kCarrierTableSize;
        float pulse = (phase < kDutyCycle) ? 1.0f : -1.0f;
        carrierTable[i] = float_to_q16(pulse);
    }
}

audio_buffer_pool_t *setup_audio() {
    static audio_format_t audioFormat = {
        .sample_freq = kSampleRateHz,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2,
    };
    static audio_buffer_format_t producerFormat = {
        .format = &audioFormat,
        .sample_stride = 4,
    };

    audio_buffer_pool_t *pool = audio_new_producer_pool(&producerFormat, 4, kBufferSamples);

    audio_i2s_config_t i2sConfig = {
        .data_pin = PICO_AUDIO_I2S_DATA_PIN,
        .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
        .dma_channel = 0,
        .pio_sm = 0,
    };

    const audio_format_t *outputFormat = audio_i2s_setup(&audioFormat, &i2sConfig);
    if (!outputFormat) {
        panic("I2S-Setup fehlgeschlagen - Pins/Format pruefen");
    }

    audio_i2s_connect(pool);
    audio_i2s_set_enabled(true);
    return pool;
}

void setup_adc() {
    adc_init();
    adc_gpio_init(POT_ADC_GPIO);
    // Einziger ADC-Kanal ist jetzt der Pot - einmal auswählen reicht.
    adc_select_input(POT_ADC_CHANNEL);
}

float read_adc_normalized() {
    uint16_t raw = adc_read();
    return (float)raw / 4095.0f;
}

// Umkehr-Schalter für den Fall, dass das Poti physisch andersherum
// verkabelt ist als die Software erwartet (siehe DEVLOG - `pot=`
// blieb bei zwei Tests trotz Drehens am selben Anschlag auf ~1000
// hängen, noch nicht abschließend bestätigt, ob das an der
// Verkabelung lag oder schlicht zweimal derselbe Anschlag getroffen
// wurde). Auf true stellen, falls sich am tiefen Poti-Anschlag
// weiterhin `pot=` nahe 1000 statt nahe 0 zeigt - kein Kabel-Umbau
// nötig, einfach hier umschalten.
constexpr bool kInvertPot = false;

// =====================================================================
// INMP441-Eingang: PIO (I2S-Master) -> DMA -> Ringpuffer
// =====================================================================

// Pegelanpassung: Das INMP441 hat keinen einstellbaren Gain, normale
// Sprache landet grob bei -50 bis -60 dBFS. Umrechnung 24 Bit -> Q16:
//   q16 = (s24 << kMicGainShift) >> 7
// kMicGainShift = 0 -> Vollausschlag des Mics = 1.0 in Q16
// jede +1 = +6 dB. Startwert 6 (+36 dB): Sprache landet damit grob in
// der Größenordnung, die der alte ADC-Pfad lieferte (Vollausschlag dort
// ebenfalls 1.0). Über micMin/micMax in der Diagnose nachkalibrieren -
// Ziel: normale Sprache mit Spitzen um ~200-500/1000. Max. 7.
constexpr int kMicGainShift = 3; // war 6 - Spitzen lagen bei ±4000-5600/1000, siehe DEVLOG

// Ringpuffer: 1024 Wörter = 4096 Bytes. Der DMA-Ring-Modus verlangt,
// dass der Puffer auf seine eigene Größe ausgerichtet ist.
constexpr uint32_t kMicRingWords    = 1024;
constexpr uint32_t kMicRingMask     = kMicRingWords - 1;
constexpr uint     kMicRingSizeBits = 12; // 2^12 Bytes = 4096
uint32_t g_micRing[kMicRingWords] __attribute__((aligned(kMicRingWords * sizeof(uint32_t))));

// Füllstand-Steuerung: Mic (PIO-Takt) und DAC (pico-extras-PIO-Takt)
// laufen beide vom Systemtakt ab, aber mit unterschiedlich gerundeten
// Teilern - sie driften minimal gegeneinander (grob ein Sample pro
// Sekunde oder weniger). Wir halten einen Vorlauf von kMicTargetFill
// Samples als Puffer gegen Schwankungen der Schleifenlaufzeit (z.B.
// printf alle 100 Puffer). Kosten: kMicTargetFill / kSampleRateHz
// zusätzliche Latenz (512 -> ~23 ms). Kann später verkleinert werden,
// falls micUnder/micOver in der Diagnose dauerhaft 0 bleiben.
constexpr uint32_t kMicTargetFill = 512;
constexpr uint32_t kMicMaxFill    = 896; // darüber: Lesezeiger nachziehen (deutlich unter Ringgröße!)

PIO      g_micPio = pio1;  // pio0 belegt pico-extras für den DAC
uint     g_micSm = 0;
int      g_micDmaChannel = -1;
uint32_t g_micReadIdx = 0;
uint32_t g_micUnderruns = 0;
uint32_t g_micOverruns = 0;

// Gleichspannungs-Nachführung bleibt erhalten (siehe DEVLOG Nachtrag
// 17) - das INMP441 hat zwar einen internen Hochpass, aber gerade in
// den ersten Sekunden nach dem Start einen deutlichen, abklingenden
// Offset (im hello_mic_test sichtbar). Kostet fast nichts.
constexpr float kMicDcTrackingMs = 300.0f;
q16 g_micDcState = 0;
q16 g_micDcUpdateRate = 0;

void setup_mic() {
    g_micSm = (uint)pio_claim_unused_sm(g_micPio, true);
    uint offset = pio_add_program(g_micPio, &i2s_mic_in_program);
    pio_sm_config c = i2s_mic_in_program_get_default_config(offset);

    // SCK (Basis) + WS (Basis+1) als side-set-Ausgänge
    sm_config_set_sideset_pins(&c, kMicSckPin);
    pio_gpio_init(g_micPio, kMicSckPin);
    pio_gpio_init(g_micPio, kMicWsPin);
    pio_sm_set_consecutive_pindirs(g_micPio, g_micSm, kMicSckPin, 2, true);
    // Weichere Flanken auf den Taktleitungen - weniger Einstreuung in
    // die restliche Schaltung, siehe DEVLOG (Übersprechen).
    gpio_set_slew_rate(kMicSckPin, GPIO_SLEW_RATE_SLOW);
    gpio_set_slew_rate(kMicWsPin, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(kMicSckPin, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(kMicWsPin, GPIO_DRIVE_STRENGTH_2MA);

    // SD als Eingang, mit Pulldown (INMP441 schaltet SD außerhalb seines
    // Slots und beim I2S-Verzögerungsbit hochohmig).
    sm_config_set_in_pins(&c, kMicSdPin);
    pio_gpio_init(g_micPio, kMicSdPin);
    gpio_pull_down(kMicSdPin);
    pio_sm_set_consecutive_pindirs(g_micPio, g_micSm, kMicSdPin, 1, false);

    // MSB zuerst, Autopush nach 32 Bit, RX-FIFO verdoppelt (TX unbenutzt).
    sm_config_set_in_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

    // Exakt 128 PIO-Takte pro Frame -> Samplerate = sys_clk / (128 * div).
    // Über die Config setzen - pio_sm_init() übernimmt den Teiler aus c
    // (ein vorheriges pio_sm_set_clkdiv() würde überschrieben).
    float div = (float)clock_get_hz(clk_sys) / ((float)kSampleRateHz * 128.0f);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(g_micPio, g_micSm, offset, &c);

    // DMA: RX-FIFO -> Ringpuffer, getaktet über den DREQ der State
    // Machine (ein Transfer pro Sample). Ring auf der Schreibseite: die
    // Schreibadresse läuft nach 4096 Bytes automatisch zum Pufferanfang
    // zurück. Transferzähler maximal - reicht bei 22,05 kHz für ~54 h,
    // mic_read_block() startet den Kanal danach einfach neu.
    g_micDmaChannel = dma_claim_unused_channel(true);
    dma_channel_config dc = dma_channel_get_default_config((uint)g_micDmaChannel);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_ring(&dc, true, kMicRingSizeBits);
    channel_config_set_dreq(&dc, pio_get_dreq(g_micPio, g_micSm, false));
    dma_channel_configure((uint)g_micDmaChannel, &dc,
                          g_micRing, &g_micPio->rxf[g_micSm],
                          0xFFFFFFFFu, true);

    pio_sm_set_enabled(g_micPio, g_micSm, true);
}

// Index des nächsten Worts, das die DMA schreiben wird.
inline uint32_t mic_write_idx() {
    uint32_t addr = (uint32_t)dma_channel_hw_addr((uint)g_micDmaChannel)->write_addr;
    return ((addr - (uint32_t)g_micRing) / sizeof(uint32_t)) & kMicRingMask;
}

inline uint32_t mic_available() {
    return (mic_write_idx() - g_micReadIdx) & kMicRingMask;
}

// Wartet, bis kMicTargetFill Samples im Ring liegen, und setzt den
// Lesezeiger an den Anfang dieses Vorlaufs. Einmal vor der Audio-
// Schleife aufrufen. Die ersten ~85 ms liefert das INMP441 ohnehin
// noch keine gültigen Daten.
void mic_prefill() {
    g_micReadIdx = mic_write_idx();
    while (mic_available() < kMicTargetFill) tight_loop_contents();
}

// Liefert n Mic-Samples als Q16 (DC-bereinigt, mit kMicGainShift
// skaliert). Aufbau eines DMA-Worts (MSB zuerst eingeschoben):
//   Bit 31     : I2S-Verzögerungsbit (kein Datenbit, per Pulldown 0)
//   Bit 30..7  : 24 Datenbits D23..D0
//   Bit 6..0   : Füllbits
void mic_read_block(q16 *out, uint32_t n) {
    if (!dma_channel_is_busy((uint)g_micDmaChannel)) {
        dma_channel_set_trans_count((uint)g_micDmaChannel, 0xFFFFFFFFu, true);
    }

    uint32_t avail = mic_available();
    if (avail > kMicMaxFill) {
        // Zu viel Vorlauf (Mic minimal schneller als DAC, oder lange
        // Unterbrechung) -> auf Sollvorlauf zurückspringen.
        g_micReadIdx = (mic_write_idx() - kMicTargetFill) & kMicRingMask;
        ++g_micOverruns;
    } else if (avail < n) {
        // Zu wenig da (Mic minimal langsamer als DAC) -> kurz warten.
        ++g_micUnderruns;
        while (mic_available() < n) tight_loop_contents();
    }

    for (uint32_t i = 0; i < n; ++i) {
        uint32_t raw = g_micRing[(g_micReadIdx + i) & kMicRingMask];
        int32_t s24 = (int32_t)(raw << 1) >> 8;            // Verzögerungsbit raus, Vorzeichen erhalten
        q16 x = (q16)((s24 * (1 << kMicGainShift)) >> 7);  // 24 Bit -> Q16 mit Gain
        g_micDcState = g_micDcState + q16_mul(g_micDcUpdateRate, x - g_micDcState);
        out[i] = x - g_micDcState;
    }
    g_micReadIdx = (g_micReadIdx + n) & kMicRingMask;
}

// Queues zur Kommunikation zwischen audioTask (ADC-Besitzer) und
// controlTask (Glättung/Mapping) - unverändert aus dem Vocoder-Projekt.
QueueHandle_t g_potRawQueue = nullptr;
QueueHandle_t g_carrierFreqQueue = nullptr;

// --- Kompressor-Parameter (siehe DEVLOG Nachtrag 12 für die
// Herleitung) - Werte sind hier NEU zu kalibrieren: erst für die
// 1-Band-Talkbox angepasst, jetzt durch die 3-Band-Summierung
// (Nachtrag 44) wieder eine andere Signal-Skala als beim
// 12-Band-Vocoder. Erster Schätzwert, kein gemessenes Optimum. ---
constexpr float kCompInputGain  = 2.0f; // zurückgesetzt (0.8 war falsche Stellschraube - beeinflusst auch die Gate-Erkennung, siehe DEVLOG)
constexpr float kCompThreshold  = 0.3f;
constexpr float kCompRatio      = 8.0f;
constexpr float kCompAttackMs   = 5.0f;
constexpr float kCompReleaseMs  = 150.0f;
constexpr float kCompMakeup     = 2.0f; // zurückgesetzt - 0.8 war nur zum Testen der (widerlegten) Lautstärke-Theorie, siehe DEVLOG Nachtrag 47

q16 g_compInputGainQ16 = 0;
q16 g_compThresholdQ16 = 0;
q16 g_compAttackCoeff = 0;
q16 g_compReleaseCoeff = 0;
q16 g_compMakeupQ16 = 0;
q16 g_compRatioInvQ16 = 0;
q16 g_compEnvelope = 0;

// --- Noise-Gate mit Hysterese (siehe DEVLOG Nachtrag 19/20/23 für die
// Herleitung, insbesondere warum Hysterese statt einer einzelnen
// Schwelle nötig war). Werte ebenfalls neu zu kalibrieren. ---
constexpr float kGateOpenThreshold  = 0.090f; // war 0.050 - Ruhepegel liegt jetzt bei 33-37/1000 (Rauschanteil im Carrier + 6 aktive Bänder), siehe DEVLOG Nachtrag 62
constexpr float kGateCloseThreshold = 0.055f; // war 0.030 - lag UNTER dem Ruhepegel, dadurch schloss das Gate nach dem Öffnen nie wieder
constexpr float kGateAttackMs   = 5.0f;
constexpr float kGateReleaseMs  = 40.0f; // war 120ms - vermutlich Hauptursache für hörbares Nachschwingen nach dem Sprechen, siehe DEVLOG

q16 g_gateOpenThresholdQ16 = 0;
q16 g_gateCloseThresholdQ16 = 0;
q16 g_gateAttackCoeff = 0;
q16 g_gateReleaseCoeff = 0;
q16 g_gateGain = 0;
bool g_gateIsOpen = false;

// Digitales Tiefpassfilter direkt am Ausgang, NACH Gate/Kompressor,
// VOR der int16-Wandlung - dämpft die scharfen oberen Harmonischen
// des schmalen Pulszug-Carriers (20% Duty-Cycle), die bei hohen
// Tonhöhen (nahe kMaxCarrierHz) als unangenehm hochfrequent/schrill
// auffielen (siehe DEVLOG). Bewusst SOFTWARE statt einer zusätzlichen
// Schaltung - reines Klangformungs-Problem, keine Rückwirkung auf die
// erst kürzlich stabilisierte Mic-/Gate-/Kompressor-Kette, und ohne
// das Risiko eines weiteren wackligen Steckbrett-Bauteils.
// kOutputLowpassFreqHz ist ein erster Schätzwert (dämpft grob ab dem
// Bereich, wo das Ohr am empfindlichsten auf "schrill" reagiert),
// kein gemessenes/gehörtes Optimum - nach Gehör nachjustieren.
constexpr float kOutputLowpassFreqHz = 3000.0f;
// DIAGNOSE-SCHALTER: false = Tiefpass komplett umgehen, um zu testen,
// ob er die Ursache für das beobachtete "Gate bleibt hängen" ist
// (Verdacht auf verstärkte DAC->Mic-Rückkopplung, siehe DEVLOG).
constexpr bool kOutputLowpassEnabled = false;
q16 g_outputLowpassCoeff = 0;
q16 g_outputLowpassState = 0;

void controlTask(void *) {
    float smoothedHz = 220.0f;
    for (;;) {
        float potNorm;
        if (xQueueReceive(g_potRawQueue, &potNorm, pdMS_TO_TICKS(50)) == pdTRUE) {
            float targetHz = kMinCarrierHz + potNorm * (kMaxCarrierHz - kMinCarrierHz);
            smoothedHz += (targetHz - smoothedHz) * 0.2f;
            xQueueOverwrite(g_carrierFreqQueue, &smoothedHz);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void audioTask(void *) {
    build_carrier_table();
    // Geometrische Verteilung exakt wie in vocoderBands.ts
    // (vocoderBandFrequencies()) - t=i/(N-1), freq = FREQ_MIN *
    // (FREQ_MAX/FREQ_MIN)^t. Attack/Release-Staffelung (tief=träger,
    // hoch=flinker) bleibt zusätzlich erhalten (in der Referenz nicht
    // vorhanden, da Web Audio keine Fixed-Point-Zeitkonstanten-
    // Vorberechnung braucht - für uns weiterhin sinnvoll).
    for (int b = 0; b < kNumBands; ++b) {
        float t = (kNumBands == 1) ? 0.0f : (float)b / (float)(kNumBands - 1);
        float freq = kBandFreqLowHz * powf(kBandFreqHighHz / kBandFreqLowHz, t);
        float attackMs = kAttackMsLow + t * (kAttackMsHigh - kAttackMsLow);
        float releaseMs = kReleaseMsLow + t * (kReleaseMsHigh - kReleaseMsLow);
        bands[b].init(freq, kBandQ, attackMs, releaseMs, (float)kSampleRateHz);
    }
    // Synthese-Filter separat mit breiterem Q neu konfigurieren, siehe
    // Erklärung bei kSynthesisQ oben - dieselben Frequenzen wie oben,
    // nur mit anderem Q.
    for (int b = 0; b < kNumBands; ++b) {
        float t = (kNumBands == 1) ? 0.0f : (float)b / (float)(kNumBands - 1);
        float freq = kBandFreqLowHz * powf(kBandFreqHighHz / kBandFreqLowHz, t);
        bands[b].synthesisFilter.setBandpass(freq, kSynthesisQ, (float)kSampleRateHz);
    }
    g_carrierNoiseMixQ16 = float_to_q16(kCarrierNoiseMix);
    g_noiseShapeCoeff = float_to_q16(1.0f - expf(-2.0f * (float)M_PI * kNoiseShapeFreqHz / (float)kSampleRateHz));
    for (int b = 0; b < kNumBands; ++b) {
        g_bandOutputGainQ16[b] = float_to_q16(kBandOutputGain[b]);
    }
    g_micDcUpdateRate = kQ16One - float_to_q16(expf(-1.0f / (0.001f * kMicDcTrackingMs * (float)kSampleRateHz)));

    g_compInputGainQ16 = float_to_q16(kCompInputGain);
    g_compThresholdQ16 = float_to_q16(kCompThreshold);
    g_compAttackCoeff = float_to_q16(expf(-1.0f / (0.001f * kCompAttackMs * (float)kSampleRateHz)));
    g_compReleaseCoeff = float_to_q16(expf(-1.0f / (0.001f * kCompReleaseMs * (float)kSampleRateHz)));
    g_compMakeupQ16 = float_to_q16(kCompMakeup);
    g_compRatioInvQ16 = float_to_q16(1.0f / kCompRatio);
    g_gateOpenThresholdQ16 = float_to_q16(kGateOpenThreshold);
    g_gateCloseThresholdQ16 = float_to_q16(kGateCloseThreshold);
    g_gateAttackCoeff = float_to_q16(expf(-1.0f / (0.001f * kGateAttackMs * (float)kSampleRateHz)));
    g_gateReleaseCoeff = float_to_q16(expf(-1.0f / (0.001f * kGateReleaseMs * (float)kSampleRateHz)));
    g_outputLowpassCoeff = float_to_q16(1.0f - expf(-2.0f * (float)M_PI * kOutputLowpassFreqHz / (float)kSampleRateHz));

    // Reihenfolge wichtig: setup_audio() belegt DMA-Kanal 0 und pio0 für
    // den DAC, setup_mic() holt sich danach freie Ressourcen (pio1,
    // nächster freier DMA-Kanal).
    audio_buffer_pool_t *pool = setup_audio();
    setup_adc();
    setup_mic();
    mic_prefill();

    // Ein Puffer Mic-Samples pro Ausgabepuffer, vor der Sample-Schleife
    // komplett aus dem Ring geholt.
    static q16 sMicBlock[kBufferSamples];

    uint32_t phase = 0;
    float carrierHz = 220.0f;

    // --- Diagnose-Zustand (persistiert über Puffergrenzen hinweg) ---
    constexpr uint32_t kBufferBudgetUs = (kBufferSamples * 1000000ull) / kSampleRateHz;
    static uint32_t sBufferCount = 0;
    static uint64_t sSumUs = 0;
    static uint32_t sMaxUs = 0;
    // NEU: I2S-Pufferwartezeit (take_audio_buffer) separat von der
    // reinen Sample-Verarbeitung gemessen - bisher war das in der
    // Diagnose gar nicht sichtbar. Soll klären, ob die beobachtete
    // Timing-Anomalie während der Rückkopplungs-Episoden aus dem
    // Warten auf den I2S-Puffer kommt (würde auf gestörte I2S-Taktung
    // hindeuten) oder aus der Sample-Schleife selbst (eher auf
    // Interrupt-Unterbrechungen hindeutend), siehe DEVLOG.
    static uint64_t sWaitSumUs = 0;
    static uint32_t sWaitMaxUs = 0;
    static q16 sMicMinQ16 = kQ16One;
    static q16 sMicMaxQ16 = -kQ16One;
    // Drei einzelne Band-Hüllkurven statt einer aggregierten - hat sich
    // im 12-Band-Projekt (dortiger Nachtrag 22/23) als der Diagnose-
    // Wert herausgestellt, der tatsächlich zeigt, ob die Bänder
    // spektral differenzieren, statt nur die Gesamtlautstärke.
    // Array statt fester Einzelvariablen - wächst automatisch mit
    // kNumBands mit, kein Handeintrag mehr nötig beim nächsten
    // Hochskalieren (siehe DEVLOG).
    static q16 sBandMaxQ16[kNumBands] = {};
    // Mittelwert der Band-Hüllkurven über das Diagnosefenster (Summe
    // über alle Samples) - für Messungen mit Rosa Rauschen / Stille /
    // Vokalen. Das Maximum oben springt bei Rauschen zu stark, um daraus
    // einen Frequenzgang oder Grundpegel abzulesen.
    static int64_t sBandSum[kNumBands] = {};
    static int sLastPotPermille = 0;
    static int sLastCarrierHzInt = 0;
    static int sLastGateEnvPermille = 0;
    static int sLastGateGainPermille = 0;
    // NEU: zeigt den nachverfolgten Gleichspannungs-Arbeitspunkt des
    // Mic-Eingangs (g_micDcState) an - soll klären, ob sich der
    // Arbeitspunkt während einer Rückkopplungs-Episode verschiebt,
    // ohne dass dafür live am laufenden Gerät gemessen werden muss.
    static int sLastMicDcPermille = 0;
    // Mic-Ringpuffer: Füllstand (Soll kMicTargetFill) und Anzahl der
    // Korrekturen seit Start - sollten im Normalbetrieb 0 oder sehr
    // selten sein. Häufige Korrekturen = Schleife zu langsam bzw. Takte
    // driften stärker als erwartet.
    static uint32_t sLastMicFill = 0;

    for (;;) {
        float potNorm = read_adc_normalized();
        if (kInvertPot) potNorm = 1.0f - potNorm;
        xQueueOverwrite(g_potRawQueue, &potNorm);
        xQueueReceive(g_carrierFreqQueue, &carrierHz, 0);
        uint32_t phaseInc = (uint32_t)((carrierHz * kCarrierTableSize / (float)kSampleRateHz) * 65536.0f);

        uint64_t waitStartUs = time_us_64();
        audio_buffer_t *buf = take_audio_buffer(pool, true);
        uint64_t waitUs = time_us_64() - waitStartUs;
        sWaitSumUs += waitUs;
        if ((uint32_t)waitUs > sWaitMaxUs) sWaitMaxUs = (uint32_t)waitUs;
        int16_t *samples = (int16_t *)buf->buffer->bytes;

        sLastPotPermille = (int)(potNorm * 1000.0f);
        sLastCarrierHzInt = (int)carrierHz;
        uint64_t loopStartUs = time_us_64();

        mic_read_block(sMicBlock, kBufferSamples);

        for (uint32_t i = 0; i < kBufferSamples; ++i) {
            q16 micQ16 = sMicBlock[i];
            if (micQ16 < sMicMinQ16) sMicMinQ16 = micQ16;
            if (micQ16 > sMicMaxQ16) sMicMaxQ16 = micQ16;

            q16 carrierQ16 = carrierTable[(phase >> 16) & (kCarrierTableSize - 1)];
            q16 noiseQ16 = next_noise_q16();
            g_noiseShapeState = g_noiseShapeState + q16_mul(g_noiseShapeCoeff, noiseQ16 - g_noiseShapeState);
            q16 synthCarrierQ16 = carrierQ16 + q16_mul(g_carrierNoiseMixQ16, g_noiseShapeState - carrierQ16);
            q16 mixed = 0;
            for (int b = 0; b < kNumBands; ++b) {
                bands[b].analyze(micQ16);
                if (kSoloBand < 0 || kSoloBand == b) {
                    mixed += q16_mul(bands[b].synthesize(synthCarrierQ16), g_bandOutputGainQ16[b]);
                }
            }
            for (int b = 0; b < kNumBands; ++b) {
                if (bands[b].envelope > sBandMaxQ16[b]) sBandMaxQ16[b] = bands[b].envelope;
                sBandSum[b] += bands[b].envelope;
            }

            // Level -> Kompressor -> Makeup statt hartem Clipping.
            mixed = q16_mul(mixed, g_compInputGainQ16);
            q16 absMixed = (mixed < 0) ? -mixed : mixed;
            q16 compCoeff = (absMixed > g_compEnvelope) ? g_compAttackCoeff : g_compReleaseCoeff;
            g_compEnvelope = g_compEnvelope + q16_mul(kQ16One - compCoeff, absMixed - g_compEnvelope);
            q16 compGain = kQ16One;
            if (g_compEnvelope > g_compThresholdQ16) {
                q16 excess = g_compEnvelope - g_compThresholdQ16;
                q16 compressedLevel = g_compThresholdQ16 + q16_mul(excess, g_compRatioInvQ16);
                compGain = (q16)(((int64_t)compressedLevel << kQ16Frac) / g_compEnvelope);
            }
            mixed = q16_mul(mixed, compGain);
            mixed = q16_mul(mixed, g_compMakeupQ16);

            // Noise-Gate mit Hysterese (Schmitt-Trigger-Muster).
            q16 gateThreshold = g_gateIsOpen ? g_gateCloseThresholdQ16 : g_gateOpenThresholdQ16;
            bool gateShouldBeOpen = (g_compEnvelope > gateThreshold);
            g_gateIsOpen = gateShouldBeOpen;
            q16 gateTarget = gateShouldBeOpen ? kQ16One : 0;
            q16 gateCoeff = (gateTarget > g_gateGain) ? g_gateAttackCoeff : g_gateReleaseCoeff;
            g_gateGain = g_gateGain + q16_mul(kQ16One - gateCoeff, gateTarget - g_gateGain);
            mixed = q16_mul(mixed, g_gateGain);

            // Ausgangs-Tiefpass gegen die scharfen Carrier-Obertöne
            // bei hohen Tonhöhen, siehe Erklärung oben.
            if (kOutputLowpassEnabled) {
                g_outputLowpassState = g_outputLowpassState +
                    q16_mul(g_outputLowpassCoeff, mixed - g_outputLowpassState);
                mixed = g_outputLowpassState;
            }

            if (mixed > kQ16One) mixed = kQ16One;
            if (mixed < -kQ16One) mixed = -kQ16One;

            int32_t s32 = (int32_t)(((int64_t)mixed * 32767) >> kQ16Frac);
            if (s32 > 32767) s32 = 32767;
            if (s32 < -32768) s32 = -32768;
            int16_t s = (int16_t)s32;
            samples[2 * i]     = s;
            samples[2 * i + 1] = s;
            phase += phaseInc;
        }

        buf->sample_count = buf->max_sample_count;
        give_audio_buffer(pool, buf);

        uint64_t elapsedUs = time_us_64() - loopStartUs;
        sSumUs += elapsedUs;
        if ((uint32_t)elapsedUs > sMaxUs) sMaxUs = (uint32_t)elapsedUs;
        sLastGateEnvPermille = (int)(((int64_t)g_compEnvelope * 1000) / kQ16One);
        sLastGateGainPermille = (int)(((int64_t)g_gateGain * 1000) / kQ16One);
        sLastMicDcPermille = (int)(((int64_t)g_micDcState * 1000) / kQ16One);
        sLastMicFill = mic_available();

        if (++sBufferCount >= 100) {
            int micMinPermille = (int)(((int64_t)sMicMinQ16 * 1000) / kQ16One);
            int micMaxPermille = (int)(((int64_t)sMicMaxQ16 * 1000) / kQ16One);
            printf("Diagnose[%s %s]: avg=%luus max=%luus wait=%luus waitMax=%luus budget=%luus pot=%d/1000 carrierHz=%d micMin=%d/1000 micMax=%d/1000 micDc=%d/1000 gateEnv=%d/1000 gateGain=%d/1000 (100 Puffer)\n",
                   __DATE__, __TIME__,
                   (unsigned long)(sSumUs / sBufferCount), (unsigned long)sMaxUs,
                   (unsigned long)(sWaitSumUs / sBufferCount), (unsigned long)sWaitMaxUs,
                   (unsigned long)kBufferBudgetUs,
                   sLastPotPermille, sLastCarrierHzInt, micMinPermille, micMaxPermille,
                   sLastMicDcPermille, sLastGateEnvPermille, sLastGateGainPermille);
            // Separate Zeile für die Band-Hüllkurven - Anzahl folgt
            // automatisch kNumBands, kein Umschreiben mehr nötig beim
            // nächsten Hochskalieren.
            printf("  mic: fill=%lu/%lu under=%lu over=%lu\n",
                   (unsigned long)sLastMicFill, (unsigned long)kMicTargetFill,
                   (unsigned long)g_micUnderruns, (unsigned long)g_micOverruns);
            printf("  bands[0..%d]=", kNumBands - 1);
            for (int b = 0; b < kNumBands; ++b) {
                int bandPermille = (int)(((int64_t)sBandMaxQ16[b] * 1000) / kQ16One);
                printf("%d ", bandPermille);
            }
            printf("\n");
            // Mittelwerte in 1/10000 (feiner als die Promille-Skala von
            // bands[] - leise Bänder liegen sonst alle bei 0-5).
            const int64_t kSamplesInWindow = (int64_t)sBufferCount * kBufferSamples;
            printf("  bandsAvg[0..%d] (x/10000)=", kNumBands - 1);
            for (int b = 0; b < kNumBands; ++b) {
                int avgPerTenThousand = (int)((sBandSum[b] * 10000) / ((int64_t)kQ16One * kSamplesInWindow));
                printf("%d ", avgPerTenThousand);
            }
            printf("\n");
            sBufferCount = 0;
            sSumUs = 0;
            sMaxUs = 0;
            sWaitSumUs = 0;
            sWaitMaxUs = 0;
            sMicMinQ16 = kQ16One;
            sMicMaxQ16 = -kQ16One;
            for (int b = 0; b < kNumBands; ++b) {
                sBandMaxQ16[b] = 0;
                sBandSum[b] = 0;
            }
        }

        // WICHTIG - PRIORITY-STARVATION-FIX (siehe DEVLOG Nachtrag 8):
        // audioTask hat sonst keinen echten FreeRTOS-Blockierpunkt,
        // controlTask bekäme ohne das nie CPU-Zeit.
        static uint32_t sYieldCounter = 0;
        if (++sYieldCounter >= 2) {
            sYieldCounter = 0;
            vTaskDelay(1);
        }
    }
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    panic("Stack-Overflow in Task: %s", pcTaskName);
}

extern "C" void vApplicationMallocFailedHook(void) {
    panic("FreeRTOS malloc fehlgeschlagen - configTOTAL_HEAP_SIZE in FreeRTOSConfig.h zu klein?");
}

} // namespace

int main() {
    stdio_init_all();

    g_potRawQueue = xQueueCreate(1, sizeof(float));
    g_carrierFreqQueue = xQueueCreate(1, sizeof(float));
    if (!g_potRawQueue || !g_carrierFreqQueue) {
        panic("Queue-Erstellung fehlgeschlagen (Heap zu klein?)");
    }

    // Rechenlast mit 3 Bändern weiterhin gering genug (statt 12-Band-
    // Filterbank) - ein einzelner Task auf Core0 reicht bequem, kein
    // Core1 mehr nötig.
    xTaskCreate(audioTask, "audio", 1024, nullptr, /*priority=*/3, nullptr);
    xTaskCreate(controlTask, "control", 512, nullptr, /*priority=*/1, nullptr);

    vTaskStartScheduler();

    panic("vTaskStartScheduler() zurueckgekehrt - Heap zu klein?");
    return 0;
}