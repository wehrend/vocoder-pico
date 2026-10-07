#pragma once
// =====================================================================
// display.h - 4-stellige 7-Segment-Anzeige mit TM1637
// =====================================================================
// Anschluss (siehe config.h): CLK -> GP20 (Pin 26), DIO -> GP21 (Pin 27),
// VCC -> 3V3(OUT) (Pin 36) - NICHT 5 V: die Module haben Pullups von
// CLK/DIO nach VCC, bei 5 V lägen 5 V an den GPIOs.
// Das Modul ist eine "Uhren"-Anzeige: Doppelpunkt zwischen Stelle 2 und 3,
// keine Dezimalpunkte - der Doppelpunkt dient als Komma.
//
// Nur aus dem controlTask aufrufen (Bit-Banging mit kurzen Wartezeiten,
// ~1 ms pro Aktualisierung; der audioTask hat höhere Priorität und
// unterbricht jederzeit).

#include <cstdint>

void display_init();

// Formant-Verschiebung anzeigen, z.B. " 2:50" = +2.50 Bänder,
// "-1:25" = -1.25 Bänder (Doppelpunkt = Komma).
void display_show_formant(float shiftBands);

// Name der Carrier-Wellenform anzeigen (PULS, SAgE, rECt, rAUS).
void display_show_wave(uint8_t wave);