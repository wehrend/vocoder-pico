// =====================================================================
// dual_core.cpp - Core1-Schleife und FIFO-Handshake
// =====================================================================
// Zwei Phasen pro Puffer (Analyse, dann Synthese), je ein Handshake -
// nötig für den Bändertausch (siehe filterbank.h).

#include "dual_core.h"
#include "filterbank.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/sync.h"

namespace {
// Befehle an Core1 über die FIFO (pro Puffer erst Analyse, dann Synthese).
constexpr uint32_t kCmdAnalyze    = 1;
constexpr uint32_t kCmdSynthesize = 2;

// Eingangsdaten für Core1 (von Core0 vor dem jeweiligen Befehl gesetzt).
const q16 *volatile s_modulator = nullptr;
const q16 *volatile s_carrier = nullptr;
// Teilergebnisse von Core1 (obere Bänder).
q16 s_core1Mix[kBufferSamples];
q16 s_core1GateDet[kBufferSamples];

void core1_entry() {
    for (;;) {
        uint32_t cmd = multicore_fifo_pop_blocking();  // Befehl von Core0
        __dmb();                                        // Eingangsdaten sichtbar machen
        if (cmd == kCmdAnalyze) {
            filterbank_analyze(kCore1FirstBand, kNumBands, s_modulator, s_core1GateDet);
        } else {
            filterbank_synthesize(kCore1FirstBand, kNumBands, s_carrier, s_core1Mix);
        }
        __dmb();                                        // Ergebnisse vor dem Fertig-Signal sichtbar machen
        multicore_fifo_push_blocking(cmd);
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

    // Phase 1: Analyse (Hüllkurven aller Bänder in den gemeinsamen Speicher)
    __dmb();
    multicore_fifo_push_blocking(kCmdAnalyze);
    uint64_t t0 = time_us_64();
    filterbank_analyze(0, kCore1FirstBand, modulator, gateDetOut);
    uint64_t t1 = time_us_64();
    multicore_fifo_pop_blocking();          // warten, bis Core1 seine Hüllkurven fertig hat
    __dmb();
    uint64_t t2 = time_us_64();

    // Phase 2: Synthese (jedes Band liest die Hüllkurve seines Quellbands)
    multicore_fifo_push_blocking(kCmdSynthesize);
    uint64_t t3 = time_us_64();
    filterbank_synthesize(0, kCore1FirstBand, carrier, mixOut);
    uint64_t t4 = time_us_64();
    multicore_fifo_pop_blocking();
    __dmb();
    uint64_t t5 = time_us_64();

    // Teilergebnisse zusammenführen.
    for (uint32_t i = 0; i < kBufferSamples; ++i) {
        mixOut[i] += s_core1Mix[i];
        gateDetOut[i] += s_core1GateDet[i];
    }
    return { (uint32_t)((t1 - t0) + (t4 - t3)), (uint32_t)((t2 - t1) + (t5 - t4)) };
}