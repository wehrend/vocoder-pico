# DEVLOG: I2S Sine Hello World auf dem PicoADK

Ziel: Kleinstmöglicher Testaufbau, um den Audio-Signalweg (I2S →
PCM5100A) auf dem PicoADK zu verifizieren, als Vorstufe zu einem
VC16-artigen Vocoder-Projekt. Kein FreeRTOS, kein ADC - nur ein
Sinuston, um Pins, Toolchain und Sample-Rate zu bestätigen.

**Ergebnis: Erfolgreich.** Sauberer 440 Hz Ton über den Onboard-DAC.

## Hardware-Findings

### PCM5100A ist bereits verbaut, aber nicht am Header

Der PicoADK hat den I2S-DAC PCM5100A fest verbaut. Die drei I2S-
Signale sind aber **intern fest verdrahtet**, nicht am Pin-Header
herausgeführt - deswegen sind sie im normalen (Header-)Pinout-Tool
nicht zu finden. Quelle: `DatanoiseTV/PicoADK-Hardware`-Repo, Abschnitt
"Internal Signals".

| GPIO | Funktion                              |
| ---- | ------------------------------------- |
| 16   | PCM5100A I2S **DAT**                  |
| 17   | PCM5100A I2S **BCLK**                 |
| 18   | PCM5100A I2S **LRCLK**                |
| 23   | PCM5100A **DEMP** (Deemphase 44.1kHz) |
| 25   | PCM5100A **XSMT** (Mute/Unmute)       |

### Stolperfalle 1: XSMT ist standardmäßig stumm geschaltet

`XSMT` (GPIO25) muss von der Software explizit auf **HIGH** gesetzt
werden. Der Default-Zustand ist "gemutet" - ohne das bleibt der
Ausgang stumm, selbst wenn I2S-Takt und Daten technisch korrekt
ankommen. Das ist leicht mit einem generellen I2S-Konfigurationsfehler
zu verwechseln, weil beide Fälle als "Stille" erscheinen.

```cpp
gpio_init(XSMT_PIN);
gpio_set_dir(XSMT_PIN, GPIO_OUT);
gpio_put(XSMT_PIN, 1); // unmute
```

### Analog-Ausgang: Line-Pegel, kein Kopfhörerverstärker

Die mit **L**/**R** beschrifteten Pins (unten rechts am Board, USB
nach oben) sind reine Line-Ausgänge des PCM5100A. Es gibt **keinen**
dedizierten Kopfhörerverstärker-Chip auf dem Board. Für brauchbare
Kopfhörerlautstärke ist ein externer Amp nötig.

### Modulator-Eingang für den späteren Vocoder (noch nicht umgesetzt)

Der Onboard-ADC (**ADC128S102**, 8-Kanal, 12-Bit, bis 1 MS/s, an
**SPI1**: GPIO10=SCK, GPIO11=MOSI, GPIO12=MISO, GPIO13=CSn) ist für
Pots/CV ausgelegt - unipolar, kein AC-Coupling, kein Mic-Gain. Für
Stimme als Modulatorsignal wird zusätzlich ein kleiner externer
Preamp benötigt (Mic-Kapsel + 1 OpAmp-Stufe, AC-gekoppelt, auf VREF/2
vorgespannt). Das ist der nächste Hardware-Schritt.

## Toolchain-Stolperfallen

### Stolperfalle 2: ARM-Cross-Compiler fehlte

`arm-none-eabi-gcc` war initial nicht installiert. Fix unter
Debian/Ubuntu/Kali:

```bash
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi
```

### Stolperfalle 3: pico-extras falsch eingebunden

Erster Ansatz war `add_subdirectory($ENV{PICO_EXTRAS_PATH} ...)` nach
`pico_sdk_init()` - das lässt CMake fehlerfrei durchlaufen, aber ohne
dass die `pico_audio_i2s`-Bibliothek als Target entsteht (sichtbar
über `make help | grep audio`, das leer blieb). Symptom war ein
verwirrender `fatal error: pico/audio_i2s.h: No such file or
directory`, obwohl die Datei nachweislich im `pico-extras`-Checkout
vorhanden war und `PICO_EXTRAS_PATH` korrekt gesetzt war.

**Ursache:** `pico-extras` hat eine eigene Import-Datei
(`external/pico_extras_import.cmake`), analog zu
`pico_sdk_import.cmake`, die **vor** `project()` per `include()`
eingebunden werden muss - nicht per `add_subdirectory()` danach.

**Fix:**

```bash
cp $PICO_EXTRAS_PATH/external/pico_extras_import.cmake .
```

```cmake
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)   # vor project()!

project(i2s_sine_hello C CXX ASM)
...
pico_sdk_init()
# KEIN add_subdirectory für pico-extras nötig
```

**Nebenbei gelernt:** Ein zwischenzeitliches `make clean` reicht nach
einer CMake-Konfigurationsänderung nicht aus - `CMakeCache.txt` bleibt
bestehen und friert alte (falsche) Pfade ein. Bei Konfigurationsfehlern
immer `rm -rf build` und komplett neu konfigurieren.

## Build-Kommandos (funktionierend)

```bash
export PICO_SDK_PATH=/pfad/zu/pico-sdk
export PICO_EXTRAS_PATH=/pfad/zu/pico-extras

cd i2s-sine-hello
rm -rf build
mkdir build && cd build
cmake -DPICO_BOARD=pico ..
make -j4
```

Flashen: BOOT-Taste gedrückt halten, USB einstecken, `.uf2` auf das
erscheinende `RP2_BOOT`-Laufwerk kopieren.

## Nächster Schritt

Ein Carrier-Oszillator (Sägezahn/Puls) mit Frequenzsteuerung über ein
Poti am ADC128S102 (SPI1) - isoliert den ADC-Signalpfad, bevor der
Mic-Preamp für den eigentlichen Vocoder-Modulator dazukommt.

---

# DEVLOG-Nachtrag: 12-Band-Filterbank + FreeRTOS

Ziel: von 1/16-Envelope-Follower (Schritt 3) auf volle 12-Band-
Filterbank hochskalieren, und dabei von der bare-metal `while`-Schleife
auf FreeRTOS umsteigen, bevor die Komplexität (12x Analyse + 12x
Synthese pro Sample) die single-threaded Struktur sprengt.

**Status: Code geschrieben, NOCH NICHT gebaut/geflasht.** Die
folgenden Punkte sind Design-Entscheidungen mit Begründung, keine auf
Hardware verifizierten Ergebnisse - das ist der eigentliche nächste
Schritt.

## Designentscheidung: geteilter ADC zwischen zwei FreeRTOS-Tasks

Naheliegend wäre gewesen, das Poti-Handling einfach in einen eigenen
Task mit eigenem `adc_select_input()`/`adc_read()` auszulagern. Das
ist aber gefährlich: Poti und Mic hängen am selben ADC (ein
Hardware-Mux mit genau einem "aktuell ausgewählter Kanal"-Zustand).
Zwei Tasks, die unabhängig voneinander den Kanal umschalten, können
sich gegenseitig mitten im Sample-Loop den Kanal wegreißen - ohne
Fehlermeldung, nur als kaputtes/falsches Audiosignal sichtbar (schwer
zu diagnostizieren).

**Lösung:** Nur der `audioTask` fasst den ADC an (Poti 1x pro Puffer,
Mic pro Sample - wie in Schritt 3). Der rohe Poti-Wert geht per
`xQueueOverwrite()` (Länge-1-"Mailbox", kein Backlog) an den
`controlTask`, der glättet/mappt und das Ergebnis über eine zweite
Länge-1-Queue zurückgibt. `audioTask` liest die mit Timeout 0
(nicht-blockierend) - falls kein neuer Wert da ist, wird einfach der
letzte bekannte weiterverwendet, statt die Echtzeitschleife zu
blockieren.

## Designentscheidung: Sägezahn statt Sinus als Carrier

Schritt 3 hatte einen Sinuston, dessen Lautstärke von der Hüllkurve
gesteuert wurde. Für einen echten Vocoder reicht das nicht: der
Carrier muss Energie in allen 12 Analysebändern haben, sonst bleiben
Bänder mit höherer Mittenfrequenz stumm, egal wie laut gesprochen
wird. Sägezahn (naive Wavetable, kein Band-Limiting) liefert diese
Obertöne. Nebenentscheidung: Carrier-Frequenzbereich auf 80-400Hz
begrenzt (statt 80-1500Hz wie beim alten Sinus) - je höher die
Grundfrequenz, desto weniger Obertöne liegen unterhalb der oberen
Analysegrenze von 8kHz.

## Offene Risiken für den Hardware-Test

- **CPU-Budget unklar**: 12 Bänder × 2 Biquads/Sample ist ein
  Vielfaches der Rechenlast von Schritt 3, auf einem M0+ ohne FPU bei
  44.1kHz. Falls das nicht rechtzeitig fertig wird: Knacken/Aussetzer
  zu erwarten. Erste Gegenmaßnahmen bei Bedarf: Sample-Rate senken,
  Puffer vergrößern, Bandzahl testweise reduzieren um die Grenze zu
  finden.
- **FreeRTOSConfig.h ungetestet**: Taktrate/Stackgrößen/Heap sind ein
  erster plausibler Entwurf, keine verifizierten Werte.
- Falls `take_audio_buffer(pool, true)` intern busy-waited statt
  FreeRTOS-freundlich zu blockieren, könnte der niedrigpriore
  `controlTask` in der Praxis seltener drankommen als die
  20ms-`vTaskDelay` vermuten lässt - im Zweifel mit einer GPIO-Toggle+
  Oszi-Messung oder `uxTaskGetStackHighWaterMark` verifizieren, wenn
  die Poti-Reaktion auf Hardware träge wirkt.

## Nächster Schritt

Bauen, flashen, hören - und dieses Nachtrag-Kapitel um die tatsächlich
gemessenen/gehörten Ergebnisse ergänzen (wie bei den vorigen
Schritten).

---

# DEVLOG-Nachtrag 2: CPU-Budget-Messung + Umstieg auf Fixed-Point

**Ergebnis der Hardware-Messung (siehe eingebaute Diagnose in
main.cpp):** Bei float-Arithmetik brauchte die 12-Band-Verarbeitung
~195.5µs/Sample, verfügbar sind bei 44.1kHz nur ~22.7µs/Sample -
**Faktor ~8.6x zu langsam**, deutlich schlimmer als die erste grobe
Schätzung (~1.5-2x). Der ADC-Read selbst war mit ~3.7µs/Sample
unauffällig - die Bandverarbeitung war der Flaschenhals, nicht die
Peripherie.

## Wichtige Erkenntnis unterwegs: `-O3` half NICHTS gegenüber `-O0`

Vor der eigentlichen Fixed-Point-Umstellung gab es eine längere
Sackgasse: die Messwerte blieben mit `-O0` UND mit bestätigtem `-O3`
exakt identisch (~51.15ms/Puffer, auf die Mikrosekunde). Das sah erst
nach einem Build-Problem aus (falscher/alter Build im `build`-Ordner),
war es aber nicht - mehrfach verifiziert (Build-Zeitstempel direkt in
die Diagnose-Ausgabe eingebaut, `strings *.elf | grep` auf den
Diagnose-String).

Die eigentliche Erklärung: Auf einem Chip ohne FPU wird JEDE
float-Operation zu einem Aufruf einer fertigen
Software-Bibliotheksroutine (`__aeabi_fmul`, `__aeabi_fadd`, ...).
Diese Routinen selbst werden durch Compiler-Optimierung nicht
schneller - sie sind vorkompilierte Bibliotheksfunktionen, keine
Inline-Instruktionen. `-O3` optimiert nur den Code UM diese Aufrufe
herum (Registerhaltung, Inlining eigener Funktionen, Wegfall von
Redundanz) - wenn aber praktisch die gesamte Zeit in den
float-Operationen selbst steckt (wie hier), bleibt für `-O3` kaum
etwas zu tun übrig. **Lehre für später: bei Softfloat-dominiertem Code
ist "Build-Typ prüfen" ein sinnvoller erster Check, aber wenn -O0 und
-O3 identisch sind, ist das selbst schon ein Hinweis auf Softfloat als
Ursache, nicht auf einen Build-Fehler.**

## Fix: Q16.16 Fixed-Point statt float im Sample-Hot-Path

Neue Dateien `fixed_point.h`, `biquad_fixed.h`, `vocoder_band_fixed.h`

- Q16.16 (32-Bit signed, 16 Integer-/16 Nachkommabits) statt float für
  alles, was pro Sample läuft (Biquad-Verarbeitung, Gleichrichtung,
  Attack/Release, Mic-Read, Carrier-Wavetable, Ausgangs-Skalierung).
  Ganzzahl-Multiplikation ist auf dem M0+ (Hardware-MUL, 1 Takt für die
  unteren 32 Bit) um ein Vielfaches billiger als eine
  IEEE754-Software-Multiplikation.

Float bleibt bewusst dort, wo es nicht im Hot Path liegt: die
trigonometrischen Berechnungen in `setBandpass()` (`sinf`/`cosf`) und
die Attack/Release-Koeffizienten (`expf`) laufen weiterhin in float,
weil sie nur einmalig beim Start pro Band berechnet werden - dort
zählt Lesbarkeit/Genauigkeit mehr als Geschwindigkeit.

Die alten float-Header (`biquad.h`, `vocoder_band.h`) bleiben
unverändert im Projekt liegen, werden aber von `main.cpp` nicht mehr
eingebunden - als Referenz/Baseline für genau die Messung oben.

## Nächster Schritt

Auf Hardware bauen/flashen und die Diagnose-Ausgabe erneut prüfen.
Erwartung: `bands=`-Wert sollte deutlich unter die vorherigen ~195µs/
Sample fallen - ob genug für die 22.7µs-Budgetgrenze, zeigt erst die
Messung. Falls es reicht: Diagnose-Code wieder entfernen (war immer
nur als Werkzeug gedacht) und mit echtem Sprechen vor dem Mic die
Klangqualität beurteilen (Attack/Release/Q/Makeup-Gain nachjustieren).
Falls es NICHT reicht: verbleibende Hebel sind Samplerate senken
(22.05kHz) und/oder die Bänder auf beide RP2040-Kerne aufteilen.

---

# DEVLOG-Nachtrag 3: Fixed-Point maß ~3.56x, nicht genug - Nachschärfen

**Messung nach Fixed-Point-Umstellung:** `bands=14057us`/256 ≈ 54.9µs/
Sample (vorher float: 195.5µs/Sample) - Faktor 3.56x schneller, aber
das Budget bei 44.1kHz liegt bei 22.7µs/Sample. Reicht noch nicht
(Faktor ~2.5x zu wenig).

**Warum nicht mehr Speedup durch Fixed-Point allein:** Der M0+ hat nur
eine 32x32->32-Bit-Hardware-Multiplikation (1 Takt), aber KEINE
32x32->64-Bit-Multiplikation. Unser `q16_mul()` braucht aber ein
64-Bit-Zwischenergebnis (zwei Q16.16-Werte multipliziert ergeben bis zu
64 Bit) - das synthetisiert der Compiler über mehrere
Einzelinstruktionen (~10-20 Takte statt 1). Immer noch massiv billiger
als Software-Float (der ganze Umbau lohnt sich), aber eben nicht der
maximal mögliche Faktor.

## Zwei weitere, verlustfreie Optimierungen

1. **b1 ist bei unserem Bandpass-Design (RBJ-Formel, konstanter 0dB-
   Peak-Gain) strukturell IMMER exakt 0** - nicht nur zufällig klein.
   Die Multiplikation `b1*x` in `Biquad::process()` war komplett
   verschwendete Rechenzeit. Entfernt (spart 1 von 5 Multiplikationen
   pro Biquad-Aufruf, 24 Aufrufe/Sample).
2. **Envelope-Update von 2 auf 1 Multiplikation reduziert**:
   `coeff*envelope + (1-coeff)*rectified` ist algebraisch identisch zu
   `envelope + (1-coeff)*(rectified-envelope)` - exakt dasselbe
   Ergebnis, nur eine Multiplikation statt zwei.

Zusammen: ~157 -> ~121 Multiplikationen/Sample (-23%), ohne jede
Präzisionseinbuße (reine Algebra-Umformung + Entfernen einer
Nulloperation).

## Zusätzlich: Samplerate 44.1kHz -> 22.05kHz

Die 23% aus den Multiplikations-Einsparungen allein reichen laut
Abschätzung noch nicht ganz. Samplerate halbieren verdoppelt das
Zeitbudget/Sample (22.7us -> 45.4us) und bleibt für unsere Filterbank
gültig (höchste Bandfrequenz 8kHz liegt klar unter der neuen
Nyquist-Grenze von 11.025kHz).

## Nächster Schritt

Erneut messen. Falls das reicht: Diagnose-Code entfernen, mit echtem
Sprechen die Klangqualität beurteilen (Attack/Release/Q/Makeup-Gain
nachjustieren - siehe offene Frage "keine Sprache erkennbar" von
diesem Test). Falls IMMER NOCH nicht genug: Bänder auf beide
RP2040-Kerne aufteilen ist der nächste Hebel; ein Wechsel auf RP2350
(potenzielle Hardware-FPU, höherer Takt) bliebe die letzte Option,
aber dafür sehe ich aktuell noch keinen Bedarf.

---

# DEVLOG-Nachtrag 4: Von 3.9% über Budget zu (hoffentlich) komfortabler Marge

**Messung nach Multiplikations-Reduktion + 22.05kHz:** `avg=12057us`
vs. `budget=11609us` - nur noch ~3.9% drüber. Groß gegenüber der
ursprünglichen 8.6x-Überschreitung, aber ein DAUERHAFTES Defizit lässt
den Puffer-Pool trotzdem irgendwann leerlaufen (ein größerer Pool
verzögert das nur, behebt es nicht - das ist ein Raten-Problem
zwischen Produzent und Konsument, kein Jitter-Problem, das sich
wegpuffern ließe).

## Eine dritte verlustfreie Optimierung: b2 komplett eliminiert

Bei der RBJ-Cookbook-Formel für unseren Bandpass gilt nicht nur b1=0
(siehe Nachtrag 3), sondern auch **b2 == -b0, strukturell exakt**.
`b2*x` lässt sich also aus dem für `y` ohnehin schon berechneten `b0*x`
per Vorzeichenwechsel gewinnen, statt eine zweite Multiplikation zu
rechnen. Reduziert `Biquad::process()` von 4 auf 3 Multiplikationen -
zusammen mit den Nachtrag-3-Optimierungen jetzt ~97 statt
ursprünglich ~157 Multiplikationen/Sample (-38%).

## Nächster Schritt

Erneut messen - Erwartung: sollte jetzt mit spürbarer Marge unter dem
Budget liegen, nicht nur knapp. Falls ja: Diagnose-Code entfernen,
Klangqualität mit echtem Sprechen beurteilen. Falls immer noch knapp:
Bänder auf beide Kerne aufteilen, oder RP2350 - aber danach sieht es
aktuell nicht aus.

---

# DEVLOG-Nachtrag 5: CPU-Budget geschafft, jetzt Klangqualität

**CPU-Budget-Kapitel abgeschlossen:** `avg=10068us` vs.
`budget=11609us`, ~13% Marge. Reihenfolge der wirksamen Schritte für
künftige Referenz: Fixed-Point (Faktor 3.56x) -> b1/b2-Elimination
(-38% Multiplikationen) -> Samplerate 44.1kHz->22.05kHz (Budget
verdoppelt). Erst alle drei zusammen reichten.

**Erster Hördurchgang:** "vocoder-artiger Klang, aber noch keine
Sprache" - Hüllkurven-Tracking funktioniert (Carrier moduliert
hörbar), aber Verständlichkeit fehlt. Zwei vermutete Ursachen,
BEWUSST NACHEINANDER statt gleichzeitig zu testen (Lehre aus dem
CPU-Budget-Debugging: mehrere Änderungen auf einmal machen die
nächste Messung uninterpretierbar):

1. **Attack/Release/Q waren für alle 12 Bänder identisch** (3ms/100ms/
   Q=4, von 100Hz bis 8kHz). Unrealistisch für Sprache: tiefe Formanten
   bewegen sich langsam, Konsonanten (schnelle Transienten in den
   oberen Bändern) brauchen deutlich schnelleres Tracking - mit
   einheitlich 100ms Release verschmiert das zu einem tonalen "Wah".
   **Geändert:** Attack/Release jetzt linear (in log-Frequenz) von
   8ms/150ms (tiefstes Band) auf 1.5ms/40ms (höchstes Band) gekoppelt,
   Q von 4.0 auf 2.0 gesenkt (schmalere Bänder klingeln länger/
   reagieren träger).
2. **Noch nicht getestet, nächster Kandidat falls (1) nicht reicht:**
   Der Sägezahn-Carrier hat eine 1/n-Obertonabnahme - bei z.B. 200Hz
   Grundfrequenz ist der Oberton bei 8kHz (40. Harmonische) nur noch
   ~2.5% der Grundtonstärke. Die oberen Analysebänder (zuständig für
   Zischlaute wie s/sch/f) bekommen vom Carrier kaum Energie zum
   Formen - ein bekannter Schwachpunkt einfacher Sägezahn-Vocoder.
   Klassische Lösung: zweiter, rauschbasierter Carrier-Anteil für
   unstimmhafte/hochfrequente Laute.

## Nächster Schritt

Mit den neuen Attack/Release/Q-Werten hören: näher an Sprache?
Falls ja, aber noch nicht ganz da: weiter feinjustieren (Werte sind
ein erster plausibler Startpunkt, kein Ergebnis einer Hardware-Messung).
Falls kaum ein Unterschied: dann ist vermutlich tatsächlich der
Carrier-Oberton-Mangel (Punkt 2 oben) die Hauptursache, und ein
Rauschanteil im Carrier wäre der nächste (isolierte) Versuch.

---

# DEVLOG-Nachtrag 6: Rauschanteil im Carrier (Ursache 2)

**Rückmeldung nach Attack/Release/Q-Anpassung:** "noch nicht
verständlich, aber deutlich näher dran" - Fortschritt, aber nicht am
Ziel. Timing unverändert (`avg=10069us`, wie erwartet - reine
Parameteränderung ändert keine Rechenlast, guter Beleg für sauberes
Einzelvariablen-Testen).

**Nächster, wieder isoliert getesteter Schritt:** Carrier bekommt einen
Rauschanteil beigemischt (`kCarrierNoiseMix = 0.3`, 70% Sägezahn / 30%
Rauschen). Begründung siehe Nachtrag 5, Punkt 2 - Sägezahn-Obertöne
fallen mit 1/n ab, oberen Bändern (Zischlaute) fehlt Anregungsenergie.
Rauschen liefert dort breitbandige Energie ohne die Komplexität einer
echten Stimmhaft/Unstimmhaft-Erkennung (klassischer Vocoder-Ansatz,
aber deutlich aufwendiger - erst versuchen, wenn die einfache Mischung
nicht reicht).

Implementierung: `xorshift32`-PRNG (reine Ganzzahl, 3 Operationen) statt
z.B. `rand()` - passt zum Fixed-Point-Ansatz, keine floats im Hot Path.
`kCarrierNoiseMix=0.3` ist ein erster Schätzwert, kein gemessenes
Optimum - increase Richtung 1.0 = reiner Rauschcarrier (nur noch
"geflüstert" klingende Ausgabe, keine Tonhöhe mehr), Richtung 0.0 =
Stand von Nachtrag 5.

## Nächster Schritt

Hören: verständlicher? Falls ja, aber Zischlaute/Konsonanten immer
noch schwach: `kCarrierNoiseMix` schrittweise erhöhen (z.B. 0.4, 0.5)
und erneut hören - EINE Variable nach der anderen. Falls insgesamt zu
"rauschig"/Tonhöhe geht verloren: wieder senken. Falls das Mischen an
sich nicht der entscheidende Hebel war: dann bräuchte es die
"richtige" Lösung (dynamische Stimmhaft/Unstimmhaft-Erkennung aus dem
Modulatorsignal, getrennter Carrier-Pfad je nach Erkennung) - das wäre
ein größerer, eigener Schritt.

---

# DEVLOG-Nachtrag 7: Rauschanteil verifiziert, jetzt Dosierung finden

**Erste Rückmeldung bei 30% Mix:** "kein Unterschied" erkennbar -
verdächtig genug für einen Extremwert-Test statt einfach die Dosis
zu erhöhen (Lehre aus dem CPU-Debugging: bei unerwartetem Ergebnis
erst verifizieren, dann weiterschrauben).

**Diagnose-Test mit 100% Rauschen (kein Sägezahn mehr):** Ergebnis
"funktioniert super, moduliert deutlich mit der Stimme" - bestätigt:
Signalweg (PRNG -> Mix -> Synthese-Bandpässe -> Hüllkurven-Skalierung)
ist korrekt verdrahtet, kein Bug. Die 30% vorhin waren schlicht zu
wenig, um im direkten Hörvergleich aufzufallen.

Nebenbeobachtung: Hüllkurve geht bei Stille nie ganz auf 0 runter -
normales Verhalten eines Envelope-Followers ohne explizites Noise-Gate
(exponentieller Release nähert sich 0 nur asymptotisch, echtes
Mic-Signal hat immer ein Grundrauschen). Kein Bug, aktuell keine
Priorität - ggf. später ein Noise-Gate nachrüsten, falls die
Stille-Hintergrundgeräusche störend werden.

**Nächster Wert zum Testen: 40%** (0.3 -> 0.4) - Sägezahn liefert
weiterhin die Tonhöhe/das Vocoder-Timbre, Rauschen liefert Energie für
die oberen Bänder/Konsonanten, aber nicht mehr dominant wie beim
100%-Test.

## Nächster Schritt

Hören bei 40%: verständlicher als bei 30%, aber noch tonal genug (nicht
nur rauschig)? Je nach Ergebnis in kleinen Schritten weiter Richtung
50-60% oder zurück Richtung 30% tasten - EIN Wert nach dem anderen
testen, nicht mehrere Parameter gleichzeitig ändern.

---

# DEVLOG-Nachtrag 8: Poti tot - Priority-Starvation-Bug gefunden

**Symptom:** `pot=` in der Diagnose bewegt sich einwandfrei voll durch
(5 bis 1000), `carrierHz=` bleibt stur bei 220 (dem Startwert) stehen.
ADC/Hardware/Wiring damit sauber ausgeschlossen (funktioniert
nachweislich auf diesem Board, siehe früherer Hello-Sine-Test) - der
Bug liegt zwischen `controlTask` und `audioTask`.

**Ursache: Priority-Starvation, kein Logikfehler.** `audioTask`
(Priorität 3) hat in seiner gesamten Endlosschleife KEINEN einzigen
echten FreeRTOS-Blockierpunkt - `take_audio_buffer()` aus
`pico_audio_i2s` ist ursprünglich für bare-metal ohne RTOS geschrieben,
vermutlich reines Busy-Spinning, kein FreeRTOS-bewusstes Blockieren.
Bei preemptivem Scheduling wechselt der Scheduler aber NUR dann zu
einem niedriger priorisierten Task (hier: `controlTask`, Priorität 1),
wenn der aktuell laufende höher priorisierte Task ECHT blockiert -
nicht schon, wenn er "gerade nichts zu tun hat". Ohne echten
Blockierpunkt in `audioTask` bekommt `controlTask` NIEMALS CPU-Zeit,
sein `vTaskDelay(20ms)` läuft nie ab, er schreibt nie in
`g_carrierFreqQueue` - `carrierHz` bleibt für immer beim Startwert.

Das ist vermutlich seit dem allerersten FreeRTOS-Umbau (Schritt 4) so
gewesen, ist aber erst jetzt aufgefallen, weil die CPU-Timing-Diagnose
nur `audioTask`s eigene Zeiten misst und von `controlTask`s
Nichtausführung nichts mitbekommt.

**Fix:** `vTaskDelay(1)` alle 2 Puffer in `audioTask` einbauen (~23ms
Intervall, passt zur ohnehin niedrigen ~50Hz-Update-Rate des Potis).
Das zwingt `audioTask` zu einem echten, kurzen Blockierpunkt, an dem
der Scheduler zu `controlTask` wechseln KANN (nicht muss - hängt davon
ab, ob dessen eigener Timer gerade abgelaufen ist, aber bei
wiederholten Gelegenheiten alle ~23ms trifft sich das zuverlässig).

**Kosten:** ~500µs/Puffer im Schnitt (1ms alle 2 Puffer) - Marge sinkt
von ~1367µs auf ~867µs (~7.5% statt ~13%). Muss die nächste
Diagnose-Messung bestätigen.

## Nächster Schritt

Bauen/flashen, Diagnose-Zeile UND Poti-Reaktion gleichzeitig prüfen:
`carrierHz=` sollte jetzt mit dem Poti mitlaufen, `avg`/`max` sollten
weiterhin klar unter `budget` bleiben (mit etwas weniger Marge als
vorher). Falls die Marge doch zu knapp wird: Intervall von "alle 2
Puffer" auf "alle 4 Puffer" strecken (langsamere Poti-Reaktion, aber
mehr Luft).

---

# DEVLOG-Nachtrag 9: Poti bestätigt behoben, Mic-Pegel-Check, Bandbereich eingeengt

**Poti-Fix bestätigt:** `pot=4/1000` -> `carrierHz=81`,
`pot=1000/1000` -> `carrierHz=399`, Zwischenwerte laufen sauber der
0.2-Glättung folgend hinterher. Priority-Starvation-Bug erledigt.
CPU-Marge mit `vTaskDelay(1)` alle 2 Puffer: `max` liegt jetzt bei
~11230-11280us gegenüber `budget=11609us` (~3-4% statt vorher ~13%
Marge in der Spitze) - läuft, aber wenig Reserve. Falls später
Knacken auftritt: Intervall auf "alle 4 Puffer" strecken.

**Mic-Pegel-Diagnose ergänzt (`micMin`/`micMax`):** Erste Messung beim
normalen Sprechen zeigte `micMin=-1000/1000, micMax=999/1000` - volle
ADC-Aussteuerung, harte Übersteuerung. Rauschboden bei Stille
ungewöhnlich hoch (~100-170/1000) - beides deutet auf zu hohen Gain am
MAX4466-Modul. Mit größerem Mic-Abstand/leiserem Sprechen:
`micMax` sauber bei ~280-500/1000, kein Clipping mehr.

**Aber: Klangeindruck "kaum ein Unterschied zur übersteuerten
Version".** Wichtige negative Erkenntnis - Clipping war real, aber
NICHT die (alleinige) Ursache für "keine Sprache erkennbar". Damit
sind CPU-Timing, Hüllkurven-Modulation, Poti, UND jetzt Pegel/Clipping
alle einzeln durchprobiert, ohne den entscheidenden Unterschied zu
machen.

**Nächster (wieder isolierter) Test: Bandbereich von 100-8000Hz auf
200-4000Hz eingeengt** (dieselben 12 Bänder, nur andere Verteilung -
keine zusätzliche Rechenlast). Begründung: klassischer
Telefon-Frequenzbereich (300-3400Hz) reicht für Sprachverständlichkeit,
weil dort die meiste phonetische Information sitzt. Log-Staffelung
über den vorherigen breiteren Bereich (~6.3 Oktaven) verschenkte
Auflösung an Bereiche, die für Worterkennung kaum beitragen - über
~4.3 statt ~6.3 Oktaven bedeutet >30% mehr Auflösung im
entscheidenden Mittenbereich.

**Noch offen, falls das nicht reicht:** Trimmer-Poti auf dem
MAX4466-Modul dauerhaft auf niedrigeren Gain einstellen (aktuell nur
über Abstand/Lautstärke kompensiert, nicht dauerhaft gelöst) - das ist
trotzdem sinnvoll unabhängig vom Bandbereich-Test, nur bisher nicht
umgesetzt.

## Nächster Schritt

Hören: verständlicher mit dem engeren Bandbereich? Falls ja: Pegel am
Mic-Modul noch dauerhaft per Trimmer korrigieren (siehe oben), dann
weiter feintunen. Falls kaum Unterschied: dann bräuchte es vermutlich
doch die aufwendigere Lösung (echte Stimmhaft/Unstimmhaft-Erkennung
statt fester Rauschmischung, oder mehr/andere Bandverteilung als
reine Log-Staffelung, z.B. Mel-/Bark-Skala).

---

# DEVLOG-Nachtrag 10: Bandbereich-Test negativ - dynamische Stimmhaft/Unstimmhaft-Erkennung

**Bandbereich 100-8000Hz -> 200-4000Hz:** "kaum ein Unterschied".
Damit sind jetzt CPU-Timing, Hüllkurven-Modulation, Poti, Mic-Pegel/
Clipping UND Bandverteilung alle einzeln durchprobiert, ohne die
entscheidende Wirkung zu zeigen - ein klares Muster, dass die Decke
für reines Parameter-Tuning einer STATISCHEN Sägezahn/Rausch-Mischung
erreicht ist.

**Schritt, der seit Nachtrag 5 mehrfach als "aufwendiger, aber
eigentlich richtig" vertagt wurde: dynamische Stimmhaft/Unstimmhaft-
Erkennung.** Begründung: mit festem Rauschanteil (zuletzt 40%) bekommt
JEDER Laut denselben Kompromiss - Vokale (bräuchten sauberen tonalen
Carrier) werden vom Rauschen vermatscht, Konsonanten (bräuchten mehr
Rauschenergie) kriegen immer noch zu wenig. Ein statischer Mix kann
diesen Zielkonflikt nicht auflösen, egal wie er dosiert wird - erklärt
das Muster "jede Dosierung ändert wenig" ziemlich gut im Nachhinein.

**Implementiert: Nulldurchgangsraten-basierte (Zero-Crossing-Rate,
ZCR) Erkennung.** Stimmhafte Laute (Vokale) sind tieffrequent-
periodisch -> wenige Nulldurchgänge. Unstimmhafte Laute (s/sch/f)
sind rauschartig-hochfrequent -> viele Nulldurchgänge. Kostenlos im
Hot Path (nur ein Vorzeichenvergleich pro Sample, der ohnehin schon
gelesene `micQ16`-Wert wird wiederverwendet). Pro Puffer ausgewertet
(nicht pro Sample - Stimmhaft/Unstimmhaft ändert sich nicht so
schnell), mit 1 Puffer (~11.6ms) Verzögerung auf den nächsten Puffer
angewendet - unhörbar, vermeidet aber eine zweite Mic-Auswertung
mitten im Sample-Loop.

`kZcrLow=0.02`/`kZcrHigh=0.12` sind Schätzwerte aus einer groben
Überschlagsrechnung (typische ZCR-Bereiche für stimmhafte vs.
unstimmhafte Sprache bei 22.05kHz, umgerechnet auf Kreuzungen pro
256-Sample-Puffer) - KEINE gemessenen Optima. Die neue
Diagnose-Ausgabe (`zcr=`/`noiseMix=`) macht das direkt beobachtbar,
statt blind zu tunen.

## Nächster Schritt (ÜBERHOLT - siehe Nachtrag 11)

~~Hören UND `zcr=`/`noiseMix=` in der Diagnose beobachten, während
abwechselnd Vokale ("aaaa") und Zischlaute ("sssss") gesprochen
werden.~~ Wurde nicht mehr verfolgt - Wehrend hat stattdessen seinen
funktionierenden Software-Vocoder als Referenz geteilt (Nachtrag 11),
der die ZCR-Erkennung komplett ersetzt hat, bevor dieser Kalibrierungs-
schritt überhaupt gemacht wurde. `zcr=` gibt es im aktuellen Code
nicht mehr.

---

# DEVLOG-Nachtrag 11: Referenz-Software-Vocoder liefert echte Antworten statt weiterer Vermutungen

Wehrend hat vier Dateien aus seinem browserbasierten Software-Vocoder
([[modular-synth]]) geteilt (`VocoderAnalysisNode.tsx`,
`VocoderBands.ts`, `VocoderSynthNode.tsx`, `VoicedUnvoicedNode.tsx`) -
**nachweislich funktionierend**. Das ändert die Debugging-Strategie:
statt weiter einzelne eigene Vermutungen zu testen, direkt die
erprobten Werte/Methoden übernehmen.

## Zwei konkrete, direkt widerlegte Annahmen

1. **Q=2.0 war falsch begründet.** Ich hatte Q von 4.0 auf 2.0 gesenkt
   mit der Annahme "schmaler = klingelt mehr/reagiert träger". Der
   funktionierende Referenz-Vocoder nutzt **Q=5** - höher als sogar
   unser ursprünglicher Wert. Zurückgesetzt auf Q=5.
2. **Die Bandbereich-Einengung (100-8000Hz -> 200-4000Hz, Nachtrag 10)
   war ebenfalls nicht in Richtung der funktionierenden Lösung.** Referenz
   nutzt 90-6000Hz über 10 Bänder (`VocoderBands.ts`) - näher am
   ursprünglichen Bereich. Übernommen: 10 Bänder, 90-6000Hz, Q=5.

## Voiced/Unvoiced-Erkennung komplett ersetzt

Die ZCR-Schätzung (Nachtrag 10) hatte nie kalibrierte Schwellwerte und
wurde nie als funktionierend bestätigt. `VoicedUnvoicedNode.tsx` zeigt
eine bewährte Alternative: Signal bei 1.5kHz in Tiefton-/Hochton-Anteil
aufteilen (Lowpass + Rest-als-Highpass), je einen geglätteten Pegel
verfolgen (~15ms Zeitkonstante), Differenz (Hochton minus Tiefton)
als Stimmhaft/Unstimmhaft-Entscheidung, mit einer zusätzlichen
Glättung (~10ms) gegen Klicks beim Umschalten. Läuft jetzt audio-rate
(pro Sample) statt wie vorher pro Puffer - genauer, und die
komplette Erkennung kostet nur ~4 zusätzliche Multiplikationen/Sample
(Lowpass-Update + 2x Hüllkurven-Update + Crossfade-Glättung),
angesichts der ~97 Multiplikationen/Sample der Filterbank vernachlässigbar.

**Noch NICHT übernommen, möglicher nächster Schritt falls das hier
nicht reicht:** Die Referenz hat eine deutlich durchdachtere
Pegel-Kette als wir - expliziter ×100-Boost der Hüllkurve vor der
VCA-Modulation, dann ×10-Level + echter Kompressor (Threshold -35dB,
Ratio 8:1, Attack 5ms, Release 150ms) + ×6 Makeup, statt unserem
einzelnen festen Makeup-Gain + hartem Clipping am Summenausgang. Ein
Kompressor statt Hard-Clip würde Verzerrungen bei lauten Passagen
vermeiden, ohne die spektrale Differenzierung zwischen Bändern zu
zerstören - das ist mit unserer bestehenden Fixed-Point-
Hüllkurven-Infrastruktur (gleiches Attack/Release-Prinzip, nur auf den
Summenausgang statt pro Band angewendet) machbar, aber ein eigener,
nicht-trivialer nächster Schritt.

## Nächster Schritt

Bauen/flashen, hören - und dabei abwechselnd "aaaa" (erwartung:
`noiseMix` nahe 0) und "sssss" (Erwartung: `noiseMix` nahe 1000)
sprechen, um die neue Erkennung zu verifizieren, bevor der
Gesamtklang beurteilt wird. Falls das Zusammenspiel aus Q=5/10 Bändern/
90-6000Hz/spektraler Erkennung den Durchbruch bringt: prima, dann
weiter feinjustieren. Falls nicht: der Kompressor/Pegel-Ketten-Umbau
oben ist der nächste, aus der Referenz abgeleitete Kandidat.

---

# DEVLOG-Nachtrag 12: Kompressor statt hartem Clipping

**Test von Nachtrag 11 (Q=5/10 Bänder/90-6000Hz):** CPU-Budget
deutlich komfortabler (10 statt 12 Bänder), aber Klangqualität
"überhaupt nicht wie die Software-Variante". `noiseMix` blieb in der
Stichprobe konstant bei 0 - unklar ob Bug oder einfach kein gezielter
Unstimmhaft-Test (keine bewusst gesprochenen Zischlaute in der Probe).
`micMax` zeigte in mehreren Zeilen wieder 999-1000/1000 - die
Mic-Übersteuerung vom Pegel-Test (Nachtrag 9) ist zurück, vermutlich
weil seitdem nur über Abstand/Lautstärke kompensiert wurde statt den
Trimmer am MAX4466 dauerhaft runterzudrehen. **Das ist jetzt kein
Kann-man-später-machen mehr** - ein geclipptes Eingangssignal verfälscht
Analyse-Hüllkurven UND den Voiced/Unvoiced-Detektor gleichermaßen und
macht jede weitere Software-Diagnose unzuverlässig.

**Wichtiger Rahmen:** Ein Vergleich "RP2040 + billiges Electret-Mic +
kleiner Lautsprecher" vs. "Software-Vocoder im Browser mit Kopfhörern"
wird nie 1:1 gleich klingen (ADC-Rauschen, Mic-Kapsel-Qualität,
DAC-Qualität) - das RP2040-Ziel ist "klar verständliche Sprache",
nicht "klingt exakt wie die Software".

**Größter noch unadressierter struktureller Unterschied, jetzt
umgesetzt:** `VocoderSynthNode.tsx` nutzt einen echten Kompressor
(Threshold -35dB, Ratio 8:1, Attack 5ms, Release 150ms) nach der
Bandsumme, statt hartem Clipping wie bei uns. Ein harter Clip
zerschneidet Signalspitzen abrupt (metallisch/hart), ein Kompressor
drückt sie sanft zusammen und erhält mehr Feinstruktur.

**Implementiert:** Level (`kCompInputGain`) -> Kompressor
(Hüllkurfenverfolgung + lineare Ratio-Kompression oberhalb
`kCompThreshold`, EINE Division/Sample für den sich dynamisch
ändernden Kompressions-Gain - alles andere vorberechnet) -> Makeup
(`kCompMakeup`), ersetzt den alten `kMakeupGain` + Hard-Clip komplett.
Attack/Release/Ratio sind 1:1 von der Referenz übernommen (reine
Zeitkonstanten/Verhältnisse, übertragbar). `kCompInputGain`/
`kCompThreshold`/`kCompMakeup` sind dagegen NEU geschätzt, weil unsere
Pegel-Skala anders ist (kein expliziter ×100-Hüllkurven-Boost pro Band
wie in der Referenz) - keine gemessenen Optima.

**Kosten:** 1 Division + ~4 Multiplikationen pro Sample zusätzlich -
angesichts der jetzt komfortablen CPU-Marge (10 statt 12 Bänder)
unkritisch, aber die nächste Diagnose-Messung sollte das bestätigen.

## Nächster Schritt

1. **Mic-Trimmer am MAX4466 jetzt dauerhaft herunterdrehen** - nicht
   mehr nur über Abstand kompensieren. Ohne sauberen Pegel ist jede
   weitere Beurteilung unzuverlässig.
2. Erst DANACH neu bauen/flashen/hören - sonst lässt sich nicht
   sagen, ob eine Verbesserung vom Kompressor oder vom saubereren
   Pegel kommt.
3. `noiseMix` gezielt mit "aaaa" vs. "sssss" prüfen (siehe Nachtrag 11) -
   das ist in der letzten Probe nicht gezielt getestet worden.

---

# DEVLOG-Nachtrag 13: Sägezahn -> Pulszug (Carrier-Obertöne reichen nicht bis in den Formantbereich)

**Test mit Kompressor:** "klingt deutlich besser" (Kompressor hilft
gegenüber hartem Clipping), aber **"klingt sauber und tonal, aber
trotzdem nicht wie Worte - eher wie ein moduliertes Brummen"**. Diese
Beschreibung ist diagnostisch wertvoll: Lautstärke moduliert korrekt
mit der Sprache, aber die KLANGFARBE ändert sich kaum - genau das
Symptom, wenn die Hüllkurven pro Band zwar korrekt unterschiedlich
sind, der Carrier aber in den entscheidenden Bändern kaum etwas zum
Formen hat.

**Ursache, die schon in Nachtrag 5 als Risiko notiert war, aber nur
für UNSTIMMHAFTE Laute (Rauschmischung) adressiert wurde:** Sägezahn
hat 1/n-Obertonabnahme. Bei stimmhaften Lauten (Vokalen) ist
`noiseMix` aber bewusst nahe 0 (reiner Sägezahn bei 80-400Hz
Grundfrequenz) - und genau dort sitzt das eigentliche Problem: die
Vokal-UNTERSCHEIDUNG (was "a" von "i" trennt) sitzt in den Formanten
F2/F3 (~800Hz-3kHz). Bei z.B. 200Hz Grundfrequenz ist der Oberton bei
2kHz schon die 10. Harmonische - mit stark reduzierter Energie. Die
Analyse erkennt dort zwar korrekt unterschiedliche Energie pro Band,
aber der Sägezahn-Carrier hat dort fast nichts zu formen -> Ergebnis
moduliert in der Lautstärke, aber kaum in der Klangfarbe = "Brummen".

**Fix: Sägezahn durch schmalen Pulszug ersetzt** (`kDutyCycle=0.2`).
Ein schmaler Puls hat ein deutlich gleichmäßigeres/breitbandigeres
Obertonspektrum, das viel weiter in den Formantbereich hineinreicht,
bevor es abfällt - historisch der Standard-Carrier für Vocoder genau
aus diesem Grund (nicht aus der Referenz übernommen, da
`VoicedUnvoicedNode.tsx`/`VocoderSynthNode.tsx` den Carrier-Oszillator
selbst nicht zeigen - das ist unsere eigene Lücke gegenüber der
Referenz, keine bereits gelöste Stelle dort). `kDutyCycle=0.2` ist ein
erster Schätzwert (schmaler = obertonreicher/heller, aber leiser in
der Grundenergie) - keine gemessene Optimum. Kostenlos: gleicher
Wavetable-Lookup-Mechanismus wie vorher, nur andere Kurvenform, einmalig
beim Start berechnet.

## Nächster Schritt

Bauen/flashen/hören - Erwartung: hellerer, obertonreicherer Carrier-
Grundcharakter (normal/gewollt bei einem Pulszug), und wichtiger:
Vokale sollten sich jetzt eher in der KLANGFARBE unterscheiden lassen,
nicht nur in der Lautstärke. Falls `kDutyCycle=0.2` zu extrem/dünn
klingt: Richtung 0.3-0.4 (weniger hell, mehr Grundenergie) probieren.
Die drei liegengebliebenen Punkte von Nachtrag 12 (Mic-Trimmer fest
einstellen, gezielter "aaaa"/"sssss"-Test) sind weiterhin offen.

---

# DEVLOG-Nachtrag 14: Zwei-Kern-Aufteilung der Bandverarbeitung

Wehrend brachte zwei externe Referenzen ein: einen dokumentierten
16-Band-Vocoder auf Teensy 3.6 (chipaudette/BlackAddr_Projects,
Blogpost "DIY Vocoder: Robots Feelin' It") mit zwei interessanten,
aber für uns aktuell verworfenen Ideen (kaskadierte Filter pro Band =
verdoppelt die Rechenlast, andere Frequenzstaffelung = ungeprüfte
zusätzliche Vermutung) - und dann die eigentlich bessere Idee: **wir
nutzen bisher nur EINEN der beiden RP2040-Kerne.**

## Warum das der bessere Hebel ist als kaskadierte Filter

Kaskadierte Filter verbrauchen vorhandene CPU-Marge. Der zweite Kern
schafft NEUE Kapazität. Das war schon seit Nachtrag 5/6 als möglicher
Fallback vorgemerkt ("Zwei-Kern-Aufteilung"), aber bewusst
zurückgestellt, weil `audioTask` exklusiven ADC-Zugriff braucht (siehe
ganz am Anfang: "WICHTIG - GETEILTE ADC-PERIPHERIE"). Die Lösung: der
ADC-Zugriff bleibt exklusiv auf Core0 (unverändert), nur die reine
Bandverarbeitung (die keinen ADC braucht) wandert teilweise auf Core1.

## Architekturentscheidung: roher Bare-Metal-Loop auf Core1, NICHT FreeRTOS-SMP

`configNUM_CORES` bleibt bei 1. Core1 läuft über `pico_multicore`
(`multicore_launch_core1()`) als einfache Endlosschleife, komplett
außerhalb von FreeRTOS' Scheduling. Bewusst so gewählt, um NICHT eine
zweite FreeRTOS-Instanz mit neuen SMP-spezifischen Nebenläufigkeits-
Risiken einzuführen - nach ADC-Sharing (Nachtrag 4) und Priority-
Starvation (Nachtrag 8) war die Lektion: neue Nebenläufigkeit nur mit
dem kleinstmöglichen Synchronisationsumfang einführen.

## Umbau der Sample-Schleife: von "alles pro Sample verschachtelt" zu drei Durchgängen pro Puffer

Vorher: Mic-Read, Voiced/Unvoiced-Erkennung, Carrier-Bau,
Bandverarbeitung UND Kompressor liefen alle ineinander verschachtelt
in EINER Pro-Sample-Schleife. Für den Zwei-Kern-Split geht das nicht
mehr, weil Core1 einen fertigen Satz Mic-/Carrier-Werte für den GANZEN
Puffer braucht, bevor es parallel losrechnen kann. Neue Struktur pro
Puffer:

1. **Durchgang 1 (nur Core0, sequentiell):** Mic-Read + Voiced/
   Unvoiced-Zustand fortschreiben + Carrier-Sample bauen, für alle 256
   Samples - Ergebnis in `s_micSamples[]`/`s_carrierSamples[]`. MUSS
   auf Core0 bleiben (ADC-Besitz + zustandsabhängige Berechnung).
2. **Durchgang 2 (parallel):** Ein Startsignal über die Inter-Core-
   FIFO (NICHT pro Sample - nur EINMAL pro Puffer, um kein neues
   Timing-Risiko wie beim Poti-Bug zu schaffen). Core0 UND Core1
   rechnen dann gleichzeitig ihre 6 Bänder über den kompletten Puffer,
   in getrennte Arrays (`s_outputCore0[]`/`s_outputCore1[]`). Core0
   wartet danach auf Core1s Fertig-Signal.
3. **Durchgang 3 (nur Core0):** Teilergebnisse summieren, Kompressor,
   int16-Konvertierung, Ausgabe.

Geteilte Arrays zwischen den Kernen sind einfache globale Arrays -
RP2040 hat keine Cache-Kohärenz-Probleme zwischen den beiden Kernen
(direkter SRAM-Zugriff), und die Indexbereiche pro Kern überschneiden
sich nie, nur die zwei Handshake-Signale müssen synchronisiert sein.

## Bandzahl gleichzeitig zurück auf 12 (6+6)

Auf Wehrends Wunsch - die 10-Band-Reduktion von Nachtrag 11 war ohnehin
nur zur CPU-Entlastung gedacht, nicht aus klanglichen Gründen. Mit der
neuen Kapazität durch den zweiten Kern ist das kein Problem mehr.

## Diagnose erweitert

Neues Feld `prep=` (Wallzeit Durchgang 1, nur Core0) ergänzt `adc=`
(nur die reinen ADC-Konversionsaufrufe) und `bands=` (jetzt die
WALLZEIT der PARALLELEN Phase - Push bis Pop -, nicht mehr die reine
Rechenzeit eines einzelnen Kerns). `bands=` ist die Zahl, an der sich
der Nutzen der Parallelisierung ablesen lässt.

## Unverifiziertes Risiko

`multicore_launch_core1()` wird in `main()` VOR `vTaskStartScheduler()`
aufgerufen - sollte unproblematisch sein, da FreeRTOS mit
`configNUM_CORES=1` Core1 nie anfasst, aber das ist eine Annahme, keine
verifizierte Tatsache (siehe die Überraschungen mit dem RP2040-
FreeRTOS-Port in Nachtrag 1-2). Falls beim ersten Boot seltsames
Verhalten auftritt (Hängenbleiben, Hardfault beim Start), ist das der
erste Verdächtige.

## Nächster Schritt

Bauen/flashen - das ist die bisher größte strukturelle Änderung seit
dem FreeRTOS-Umbau selbst, also besonders sorgfältig prüfen:

1. Startet die Firmware überhaupt (siehe Risiko oben)?
2. `bands=` in der Diagnose - liegt es spürbar unter den vorherigen
   Einzelkern-Werten? Das bestätigt, dass die Parallelisierung
   tatsächlich greift.
3. Erst DANACH den Pulszug-Klangtest von eben nachholen (der stand
   vor dieser Änderung noch aus) - jetzt mit 12 statt 10 Bändern, also
   nicht 1:1 vergleichbar mit einer eventuell schon gehörten
   Zwischenversion.

---

# DEVLOG-Nachtrag 15: Voiced/Unvoiced-Bug gefunden - Schwelle war zu streng, nicht kaputt

**Zwei-Kern-Aufteilung bestätigt funktionierend:** `bands=4887us` bei
12 Bändern (vorher ~8250us bei 10 Bändern einkernig) - CPU-Marge jetzt
~39% (`avg=7109us` vs. `budget=11609us`). Kein Startproblem, das
unverifizierte Risiko aus Nachtrag 14 (`multicore_launch_core1()` vor
`vTaskStartScheduler()`) hat sich nicht materialisiert.

**Gezielter "aaaa"/"sssss"-Test (endlich nachgeholt) fand einen
echten Bug in der Stimmhaft/Unstimmhaft-Erkennung** - aber einen
harmloseren als befürchtet. Neue Diagnose-Felder `vuLow=`/`vuHigh=`
(rohe, ungefilterte Hüllkurvenwerte) zeigten:

- "aaaa": `vuHigh`/`vuLow`-Verhältnis ~4% (z.B. 1/19, 4/102, 1/27)
- "sssss": `vuHigh`/`vuLow`-Verhältnis ~19%, teils bis 26% (z.B. 7/27, 8/31)

Die Richtung stimmt also - `vuHigh` reagiert korrekt stärker bei
Zischlauten. Der Bug: die Vergleichslogik verlangte `vuHigh >
vuLow` (Hochton muss Tiefton ABSOLUT übersteigen, Verhältnis >100%) -
aber selbst bei deutlichem "sssss" blieb das Verhältnis bei ~20-26%,
weit unter 100%. Das ist akustisch plausibel: Zischlaute sind reine
Luftturbulenz ohne Stimmbandanregung und daher grundsätzlich leiser
als Vokale - dass ihr Hochton-Anteil den Tiefton-Anteil eines lauteren
Vokals absolut überträfe, war eine unrealistische Erwartung an die
Schwelle, keine falsche Idee bei der Methode selbst.

**Fix, datenbasiert statt neu geraten:** Vergleich von "Hochton
übersteigt Tiefton" auf "Hochton erreicht einen Mindestanteil
(`kVuRatioThreshold=0.12`) von Tiefton" umgestellt.
0.12 liegt sauber zwischen den gemessenen Bereichen (aaaa ≤6%, sssss
≥18%) - kalibriert an EINER Messung mit einer Stimme/einem Mic-Aufbau,
kein universeller Wert, aber ein echter Datenpunkt statt Rateversuch.

## Nächster Schritt

Bauen/flashen, "aaaa"/"sssss"-Test wiederholen: `noiseMix` sollte bei
"aaaa" jetzt nahe 0 bleiben, bei "sssss" klar Richtung 1000 wandern.
Falls das endlich sauber trennt: Mic-Trimmer immer noch als offener
Punkt (siehe Nachtrag 12/13) fest einstellen, dann den Gesamtklang
(Pulszug + 12 Bänder + jetzt hoffentlich funktionierende Erkennung)
neu beurteilen - das war ja der eigentliche Test, der seit Nachtrag 13
mehrfach von dazwischengekommenen Bugs unterbrochen wurde.

---

# DEVLOG-Nachtrag 16: Voiced/Unvoiced-Fix bestätigt

**Test nach der Schwellen-Korrektur:** "sss" (gehalten) erreicht
`noiseMix=996/1000` in der Mitte des Zischlauts - praktisch voll
unstimmhaft. "aaa" bleibt bei 0 (bis auf einen kurzen Ausschlag auf
572 in der allerersten Zeile, vermutlich ein natürlicher behauchter
Sprechansatz vor dem eigentlichen Vokal, kein Bug). **Erkennung
funktioniert jetzt wie vorgesehen.**

Damit ist die Liste der bekannten, bestätigten Bugs leer: CPU-Budget,
Poti/Priority-Starvation, hartes Clipping (jetzt Kompressor),
Zwei-Kern-Split, Voiced/Unvoiced-Schwelle - alle behoben und verifiziert.

**Einziger seit mehreren Nachträgen unveränderter offener Punkt:**
Mic-Pegel (`micMax` erreicht in dieser Probe wieder bis 680/1000) -
der Trimmer am MAX4466 wurde noch immer nicht fest eingestellt,
sondern nur über Abstand/Lautstärke behelfsmäßig umgangen.

## Nächster Schritt

Mic-Trimmer JETZT fest einstellen (Ziel: `micMax` bleibt bei normaler
Sprechlautstärke/Abstand stabil zwischen ~300-600/1000). Erst danach
den Gesamtklang (Pulszug-Carrier + 12 Bänder + Kompressor + jetzt
funktionierende Voiced/Unvoiced-Erkennung, alles zusammen) fair
beurteilen - mit stabilem Pegel lässt sich dieses Mal auch tatsächlich
sagen, ob eine Beobachtung an den Algorithmen liegt oder am Pegel.
