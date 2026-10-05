#pragma once
// =====================================================================
// dual_core.h - Verteilung der Filterbank auf beide Kerne
// =====================================================================
// Core1 läuft als eigene Endlosschleife AUSSERHALB von FreeRTOS und
// rechnet die obere Hälfte der Bänder (gleiches Muster wie Kapitel 5 im
// Artikel). Zwei Handshakes pro PUFFER über die SIO-FIFO:
//   Core0: push (Start) -> eigene Bänder -> pop (warten) -> zusammenführen
//   Core1: pop (warten) -> seine Bänder -> push (fertig)
// Jeder Kern rechnet NUR seine eigenen Bänder (eigene Filterzustände,
// eigene Diagnose-Einträge).
//
// VORAUSSETZUNG: configSUPPORT_PICO_SYNC_INTEROP = 0 in FreeRTOSConfig.h.
// Bei 1 installiert der FreeRTOS-Port einen eigenen FIFO-Interrupt auf
// Core0, der die Handshake-Nachrichten wegräumen würde.

#include <cstdint>
#include "config.h"
#include "fixed_point.h"

constexpr int kCore1FirstBand = kNumBands / 2;   // Core0: 0..4, Core1: 5..9

struct DualCoreTiming {
    uint32_t core0BandsUs;     // Rechenzeit der Core0-Bänder
    uint32_t core1ExtraWaitUs; // wie lange Core0 danach noch auf Core1 wartete
};

// Core1 starten (nach filterbank_init()). Wartet danach auf Arbeit.
void dual_core_start();

// Alle Bänder für einen Puffer rechnen, auf beide Kerne verteilt.
// mixOut / gateDetOut erhalten die Summe über ALLE Bänder.
DualCoreTiming dual_core_process(const q16 *modulator, const q16 *carrier,
                                 q16 *mixOut, q16 *gateDetOut);