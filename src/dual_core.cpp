// =====================================================================
// dual_core.cpp - Core1-Schleife und FIFO-Handshake
// =====================================================================
// Ausgelagert aus main.cpp, inhaltlich unverändert.

#include "dual_core.h"
#include "filterbank.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/sync.h"

namespace {
// Eingangsdaten für Core1 (von Core0 vor dem Start-Signal gesetzt).
const q16 *volatile s_modulator = nullptr;
const q16 *volatile s_carrier = nullptr;
// Teilergebnisse von Core1 (obere Bänder).
q16 s_core1Mix[kBufferSamples];
q16 s_core1GateDet[kBufferSamples];

void core1_entry() {
    for (;;) {
        multicore_fifo_pop_blocking();  // Start-Signal von Core0
        __dmb();                        // Eingangsdaten von Core0 sichtbar machen
        filterbank_process(kCore1FirstBand, kNumBands, s_modulator, s_carrier,
                           s_core1Mix, s_core1GateDet);
        __dmb();                        // Ergebnisse vor dem Fertig-Signal sichtbar machen
        multicore_fifo_push_blocking(1);
    }
}
} // namespace

void dual_core_start() {
    multicore_launch_core1(core1_entry);
}

DualCoreTiming dual_core_process(const q16 *modulator, const q16 *carrier,
                                 q16 *mixOut, q16 *gateDetOut) {
    s_modulator = modulator;
    s_carrier = carrier;
    __dmb();
    multicore_fifo_push_blocking(1);                 // Core1 starten (obere Bänder)
    uint64_t core0StartUs = time_us_64();
    filterbank_process(0, kCore1FirstBand, modulator, carrier, mixOut, gateDetOut);
    uint64_t core0EndUs = time_us_64();
    multicore_fifo_pop_blocking();                   // warten, bis Core1 fertig ist
    __dmb();
    uint64_t core1DoneUs = time_us_64();

    // Teilergebnisse zusammenführen.
    for (uint32_t i = 0; i < kBufferSamples; ++i) {
        mixOut[i] += s_core1Mix[i];
        gateDetOut[i] += s_core1GateDet[i];
    }
    return { (uint32_t)(core0EndUs - core0StartUs), (uint32_t)(core1DoneUs - core0EndUs) };
}