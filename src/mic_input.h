#pragma once
// =====================================================================
// mic_input.h - INMP441 (I2S-MEMS-Mikrofon) über PIO + DMA
// =====================================================================
// Ablauf: setup_audio() (DAC, belegt pio0 + DMA-Kanal 0) -> setup_mic()
// (holt sich pio1 + freien DMA-Kanal) -> mic_prefill() -> pro Ausgabe-
// puffer einmal mic_read_block().
// Wird nur einmal pro Puffer aufgerufen, nicht pro Sample - darum als
// eigene .cpp ohne Rechenzeit-Nachteil.

#include <cstdint>
#include "fixed_point.h"

// Sollvorlauf im Ringpuffer (Samples) als Reserve gegen Schwankungen der
// Schleifenlaufzeit; ~23 ms Latenz bei 22.05 kHz. Siehe mic_input.cpp.
constexpr uint32_t kMicTargetFill = 512;

// PIO-I2S-Empfänger + DMA-Ringpuffer starten (nach setup_audio()!).
void setup_mic();

// Wartet, bis kMicTargetFill Samples vorliegen; einmal vor der Audioschleife.
void mic_prefill();

// Liefert n Mic-Samples als Q16 (DC-bereinigt, mit kMicGainShift skaliert).
void mic_read_block(q16 *out, uint32_t n);

// Diagnose
uint32_t mic_fill();       // aktueller Füllstand des Ringpuffers (Samples)
uint32_t mic_underruns();  // Anzahl Korrekturen "zu wenig da" seit Start
uint32_t mic_overruns();   // Anzahl Korrekturen "zu viel Vorlauf" seit Start
q16      mic_dc_level();   // nachgeführter Gleichspannungsanteil