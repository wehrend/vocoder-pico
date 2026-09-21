# 12-Band-Vocoder mit FreeRTOS (generischer Pico + PCM5102A + MAX9814)

Schrittweiser Testaufbau Richtung VC16-artiger Vocoder. Portiert vom
ursprünglichen PicoADK-Aufbau auf einen generischen Raspberry Pi Pico
mit externem PCM5100A/PCM5102A-Breakout, weil weitere PicoADK-Boards
(DatanoiseTV) nicht verfügbar waren. Jeder Schritt baut auf dem vorigen
auf und verifiziert einen weiteren Baustein, bevor mehr Komplexität
dazukommt.

**Aktueller Stand: Schritt 4 - volle 12-Band-Filterbank + FreeRTOS.
NOCH NICHT AUF ECHTER HARDWARE GEBAUT/GEFLASHT** (siehe "Nächster
Schritt" unten).

## Was das Programm jetzt macht

1. **Carrier**: Sägezahn-Wavetable (statt Sinus wie in Schritt 3 - ein
   Sinus liefert nur bei einer Frequenz Energie, ein Vocoder braucht
   Obertöne in allen 12 Bändern). Tonhöhe 80–400Hz, gesteuert vom Poti.
2. **Modulator**: Mic-Signal (MAX9814/MAX4466-Ausgang) läuft durch 12
   parallele Analyse-Bandpässe (log-gestaffelt 100Hz–8000Hz, Q=4) →
   Gleichrichter → Attack(3ms)/Release(100ms) → 12 Hüllkurven.
3. Der Carrier läuft durch 12 Synthese-Bandpässe mit denselben
   Mittenfrequenzen; jedes Band wird mit seiner zugehörigen Hüllkurve
   skaliert und alle 12 Bänder werden aufsummiert → Ausgang.
4. **FreeRTOS**: Audio-I/O + alle 12 Bänder laufen in einem
   hochprioren `audioTask` (exakt wie die alte `while`-Schleife, nur
   jetzt als Task). Poti-Glättung/-Mapping läuft in einem
   niedrigprioren `controlTask`. Details und die Begründung für die
   Queue-basierte Kopplung (statt Mutex um den ADC) stehen als
   Kommentar am Anfang von `main.cpp`.
5. Sample-Rate weiterhin 44.1 kHz, Puffergröße 256 Samples (Pool auf 4
   Puffer erhöht wegen FreeRTOS-Scheduling-Jitter), Ausgabe über I2S an
   den PCM5102A.

## Offene Fragen für den Hardware-Test

- **CPU-Budget**: 12 Bänder × 2 Biquads pro Sample ist deutlich mehr
  Rechenlast als die 1-Band-Stufe, auf einem M0+ ohne FPU. Ob 44.1kHz
  mit 256er-Puffern zeitlich reicht, ist noch nicht gemessen. Falls
  Knacken/Aussetzer auftreten: Sample-Rate auf 22.05kHz senken, Puffer
  vergrößern, oder Bandzahl testweise reduzieren, um zu sehen, wo die
  Grenze liegt.
- **FreeRTOSConfig.h** (Stackgrößen, Heap, Taktrate) ist ein erster,
  ungetesteter Entwurf - siehe Kommentare direkt in der Datei.
- **Sägezahn-Aliasing**: naive (nicht band-limited) Wavetable, könnte
  bei höheren Carrier-Frequenzen hörbar/unangenehm klingen - erst mal
  akzeptiert, siehe "Nächster Schritt".

## Unterschiede zur PicoADK-Version

- I2S-Pins sind frei wählbar (unten), da kein fest verdrahteter
  interner DAC mehr vorliegt - selbst verkabelt statt onboard.
- Kein ADC128S102/SPI mehr für Poti+Mic - der native RP2040-ADC
  (GPIO26-28) übernimmt beide Kanäle.
- Kein XSMT/DEMP-GPIO-Handling mehr im Code - das übernehmen feste
  Jumper/Lötbrücken auf dem PCM5102A-Breakout selbst (siehe unten).
- Kein Debug-Logging/Heartbeat-LED mehr aktiv (war während der
  I2S-Fehlersuche drin, aktuell wieder entfernt - bei Bedarf leicht
  wieder ergänzbar, `stdio_init_all()` läuft bereits).

## Pins

### I2S zum PCM5102A - frei verkabelt, an CMake-Defines anpassen

```
GPIO16 = BCK   (PICO_AUDIO_I2S_CLOCK_PIN_BASE)
GPIO17 = LRCK  (= CLOCK_PIN_BASE + 1, automatisch)
GPIO18 = DIN   (PICO_AUDIO_I2S_DATA_PIN)
```

BCK/LRCK müssen aufeinanderfolgende GPIO-Nummern sein, DIN ist
unabhängig wählbar. Werte stehen in `CMakeLists.txt` als
`target_compile_definitions`.

### PCM5102A-Breakout: Jumper/Lötbrücken (nicht software-gesteuert!)

```
XSMT -> HIGH / "un-mute"        (sonst dauerhaft stumm, unabhängig vom Code)
FMT  -> LOW  / "I2S"            (nicht Left-Justified, sonst Rauschen statt Ton)
DEMP -> LOW  / "off"
FLT  -> LOW  / "normal latency"
SCK  -> LOW  / "kein externer Master-Clock"
```

Alle fünf sind reine Hardware-Konfiguration auf dem Modul, GPIO-seitig
passiert dafür nichts. Falls dein Modul diese Signale stattdessen als
GPIO-Pins herausführt statt sie zu jumpern, müssten sie wie bei der
PicoADK-Version per `gpio_init`/`gpio_put` gesetzt werden.

### Nativer RP2040-ADC für Poti und Mic

```
GPIO26 = ADC0 = Frequenz-Poti  (Schleifer -> GPIO26, Außenbeine -> 3V3/GND)
GPIO27 = ADC1 = Mic-Amp-Ausgang (MAX9814)
```

Kanalzuordnung in `main.cpp` als `POT_ADC_CHANNEL`/`MIC_ADC_CHANNEL`
änderbar. Wichtig im Code: `adc_select_input()` wird nur beim
tatsächlichen Kanalwechsel aufgerufen (einmal pro Puffer kurz auf
Poti, dann fest auf Mic für die gesamte Sample-Schleife) - nicht vor
jedem einzelnen Read.

### MAX9814-Verdrahtung

```
MAX9814 VDD  -> 3V3   (nicht 5V/VBUS - RP2040-ADC verträgt nur bis 3.3V)
MAX9814 GND  -> GND
MAX9814 OUT  -> GPIO27
MAX9814 GAIN -> offen lassen = 60dB (Standard) | VDD = 40dB | GND = 50dB
MAX9814 A/R  -> offen lassen = 1:4000 (Standard) | VDD = 1:2000 | GND = 1:500
```

Kein zusätzliches Bias-Netzwerk nötig - das Modul liefert bereits ein
ADC-taugliches, vorgespanntes Signal (Bias laut Datenblatt fest bei
ca. 1,25V, unabhängig von VDD). Der Bandpass im Code entfernt den
verbleibenden Rest-Bias automatisch (0 Gain bei DC), exakte
Zentrierung ist daher nicht kritisch.

### Analog-Ausgang

Line-Pegel, **kein** Kopfhörerverstärker auf dem PCM5102A-Breakout.
Kopfhörer direkt anschließen funktioniert (verbreitete DIY-Praxis),
belastet den Ausgang aber unterhalb der spezifizierten Lastimpedanz
(Datenblatt empfiehlt ≥10kΩ, Kopfhörer liegen typischerweise bei
16–300Ω). Für Dauerbetrieb empfiehlt sich ein Serienwiderstand
(100–470Ω) je Kanal vor der Klinkenbuchse - schützt den Ausgang und
senkt gleichzeitig den ohnehin oft zu hohen Line-Pegel für Kopfhörer.

## Abhängigkeiten

- [pico-sdk](https://github.com/raspberrypi/pico-sdk)
- [pico-extras](https://github.com/raspberrypi/pico-extras) (liefert `pico_audio_i2s`)
- [FreeRTOS-Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel) (RP2040-Port, `portable/ThirdParty/GCC/RP2040`)

```bash
export PICO_SDK_PATH=~/development/frontend_masterclass/abschlussprojekt/hardware/pico-sdk
export PICO_EXTRAS_PATH=~/development/frontend_masterclass/abschlussprojekt/hardware/pico-extras
export FREERTOS_KERNEL_PATH=~/development/frontend_masterclass/abschlussprojekt/hardware/FreeRTOS-Kernel
cp $PICO_EXTRAS_PATH/external/pico_extras_import.cmake .
cp $FREERTOS_KERNEL_PATH/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake .
```

Beide Import-Dateien kopieren ist zwingend nötig - sie müssen per
`include()` vor `project()` eingebunden werden, nicht per
`add_subdirectory()` danach (gleiches Prinzip wie schon bei
`pico-extras`).

## Bauen

```bash
mkdir build && cd build
cmake -DPICO_BOARD=pico ..
make -j4
```

Erzeugt `vocoder_12band_freertos.uf2`. Pico im BOOTSEL-Modus (Boot-Taste
beim Einstecken gedrückt halten) anschließen, `.uf2` draufkopieren.

## Erfolgskriterien

- Sauberer Ton ohne Knacken/Aussetzer, kein Rauschen
- Frequenz reagiert glatt auf Poti-Drehung
- Lautstärke reagiert hörbar auf Sprechen/Klatschen vor dem Mic

## Wenn es nicht funktioniert

- **Stille:** XSMT-Jumper auf HIGH prüfen; Verkabelung
  (Durchgangsprüfung stromlos) und Data-/Clock-Pins gegen die Tabelle
  oben verifizieren. Power-LED des Breakouts zeigt nur "Strom
  kommt an", nichts über die DAC-Funktion selbst.
- **Rauschen statt Ton:** FMT-Jumper prüfen (muss auf I2S/LOW stehen,
  nicht Left-Justified) - ein falsch gesetztes oder offenes FMT bringt
  genau dieses Symptom, kein Software-Fix möglich (Protokoll ist im
  PIO-Programm der Library fest verdrahtet).
- **Kein Takt/DAC bleibt tot trotz XSMT=HIGH:** SCK-Jumper prüfen,
  muss auf LOW stehen, damit das Modul seinen internen PLL aus BCK
  ableitet.
- **Falsche Tonhöhe/verzerrt:** `kSampleRateHz` muss zur tatsächlichen
  I2S-Konfiguration passen.
- **Knacken in regelmäßigen Abständen:** `kBufferSamples` erhöhen oder
  Pufferanzahl in `audio_new_producer_pool()` (aktuell 3) erhöhen.
- **Hüllkurve reagiert kaum/übersteuert sofort:** Skalierungsfaktor
  `6.0f` vor dem Clipping in `main()` ist ein grober Platzhalter -
  hängt von Mic-Gain (MAX9814 GAIN-Pin: 40/50/60dB) und Sprechabstand
  ab. Bei Bedarf `printf` für den rohen `envelope`-Wert temporär wieder
  einbauen und Faktor so einstellen, dass er bei normaler
  Sprechlautstärke nahe 1.0 bleibt, ohne dauerhaft zu clippen.
- **Bandpass scheint nichts zu tun:** Mic-Read + Filter müssen **pro
  Sample** laufen, nicht einmal pro Puffer (das Poti darf das, Audio
  nicht) - in `main.cpp` prüfen, dass das innerhalb der
  Sample-Schleife passiert.

## Nächster Schritt

1. **Auf echter Hardware bauen/flashen/messen** - CPU-Budget bei 12
   Bändern + FreeRTOS-Overhead ist bisher nur überschlagen, nicht
   gemessen. Ergebnis (und ggf. nötige Anpassungen an Sample-Rate,
   Puffergröße oder FreeRTOSConfig.h) hier oder im DEVLOG festhalten.
2. Bandzahl/Frequenzen/Q/Attack-Release nach Gehör nachjustieren,
   sobald der erste Ton auf Hardware da ist - aktuelle Werte
   (`kBandQ=4`, `kAttackMs=3`, `kReleaseMs=100`) sind ein erster,
   ungetesteter Startpunkt für alle 12 Bänder gleichzeitig.
3. Optional: Sägezahn-Aliasing reduzieren (z.B. PolyBLEP oder größere
   Wavetable), falls der Carrier auf Hardware unangenehm klingt.