// =====================================================================
// display.cpp - TM1637-Ansteuerung (Bit-Banging, Open-Drain)
// =====================================================================
// Protokoll (TM1637-Datenblatt): ähnlich I2C, aber LSB zuerst und ohne
// Adresse. Pro Aktualisierung drei Befehle:
//   0x40            Daten schreiben, Adresse automatisch weiterzählen
//   0xC0 + 4 Bytes  ab Stelle 0 die Segmente
//   0x88 | Hellig.  Anzeige an, Helligkeit 0..7
// Die Leitungen werden wie bei I2C nur auf LOW gezogen oder losgelassen
// (Eingang -> der Pullup des Moduls zieht auf HIGH). Das ACK des Chips
// wird nicht ausgewertet - ohne angeschlossenes Modul passiert also
// einfach nichts (kein Hängenbleiben).

#include "display.h"
#include "config.h"
#include "carrier_wave.h"
#include "pico/stdlib.h"
#include <cmath>

namespace {
constexpr uint32_t kBitDelayUs = 5;

// Segmentbits: a=0x01 b=0x02 c=0x04 d=0x08 e=0x10 f=0x20 g=0x40,
// 0x80 an Stelle 1 = Doppelpunkt.
constexpr uint8_t kDigits[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
constexpr uint8_t kSegMinus = 0x40;
constexpr uint8_t kSegBlank = 0x00;
constexpr uint8_t kColon    = 0x80;

// Wellenform-Namen (deutsch, 7-Segment-tauglich):
//   PULS = Impulszug, SAgE = Sägezahn, rECt = Rechteck, rAUS = Rauschen
constexpr uint8_t kWaveText[kWaveCount][4] = {
    {0x73, 0x3E, 0x38, 0x6D},   // P U L S
    {0x6D, 0x77, 0x6F, 0x79},   // S A g E
    {0x50, 0x79, 0x39, 0x78},   // r E C t
    {0x50, 0x77, 0x3E, 0x6D},   // r A U S
};

inline void line_low(uint pin)     { gpio_set_dir(pin, GPIO_OUT); }   // Ausgang, Wert 0
inline void line_release(uint pin) { gpio_set_dir(pin, GPIO_IN); }    // Pullup zieht hoch
inline void bit_delay()            { sleep_us(kBitDelayUs); }

void tm_start() {
    line_release(kDisplayClkPin);
    line_release(kDisplayDioPin);
    bit_delay();
    line_low(kDisplayDioPin);
    bit_delay();
}

void tm_stop() {
    line_low(kDisplayClkPin);
    line_low(kDisplayDioPin);
    bit_delay();
    line_release(kDisplayClkPin);
    bit_delay();
    line_release(kDisplayDioPin);
    bit_delay();
}

void tm_write_byte(uint8_t b) {
    for (int bit = 0; bit < 8; ++bit) {       // LSB zuerst
        line_low(kDisplayClkPin);
        if (b & 0x01) line_release(kDisplayDioPin); else line_low(kDisplayDioPin);
        bit_delay();
        line_release(kDisplayClkPin);
        bit_delay();
        b >>= 1;
    }
    // ACK-Takt: DIO loslassen, Chip zieht ihn kurz auf LOW (nicht ausgewertet)
    line_low(kDisplayClkPin);
    line_release(kDisplayDioPin);
    bit_delay();
    line_release(kDisplayClkPin);
    bit_delay();
    line_low(kDisplayClkPin);
    bit_delay();
}

void tm_show(const uint8_t seg[4]) {
    tm_start(); tm_write_byte(0x40); tm_stop();
    tm_start(); tm_write_byte(0xC0);
    for (int i = 0; i < 4; ++i) tm_write_byte(seg[i]);
    tm_stop();
    tm_start(); tm_write_byte((uint8_t)(0x88 | (kDisplayBrightness & 0x07))); tm_stop();
}
} // namespace

void display_init() {
    const uint pins[2] = {kDisplayClkPin, kDisplayDioPin};
    for (uint pin : pins) {
        gpio_init(pin);
        gpio_put(pin, 0);     // Ausgangswert immer 0, umgeschaltet wird die Richtung
        gpio_pull_up(pin);   // nötig: dieses Modul hat KEINE eigenen Pullups an CLK/DIO
        gpio_set_dir(pin, GPIO_IN);
    }
    const uint8_t blank[4] = {kSegBlank, kSegBlank, kSegBlank, kSegBlank};
    tm_show(blank);
}

void display_show_formant(float shiftBands) {
    int centi = (int)lroundf(shiftBands * 100.0f);    // -300 .. 300
    bool negative = centi < 0;
    if (negative) centi = -centi;
    if (centi > 999) centi = 999;
    uint8_t seg[4];
    seg[0] = negative ? kSegMinus : kSegBlank;
    seg[1] = (uint8_t)(kDigits[(centi / 100) % 10] | kColon);   // Doppelpunkt = Komma
    seg[2] = kDigits[(centi / 10) % 10];
    seg[3] = kDigits[centi % 10];
    tm_show(seg);
}

void display_show_wave(uint8_t wave) {
    if (wave >= kWaveCount) return;
    tm_show(kWaveText[wave]);
}