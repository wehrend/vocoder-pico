#pragma once
// =====================================================================
// diagnostics.h - Messwerte sammeln und alle 100 Puffer ausgeben
// =====================================================================
// Pro Sample: inline-Zähler (im Sample-Takt auf Core0).
// Pro Puffer: buffer_done() - sammelt Zeiten/Zustände und gibt alle
// kDiagWindowBuffers Puffer die Diagnosezeilen über USB-Seriell aus.

#include <cstdint>
#include "fixed_point.h"
#include "dual_core.h"
#include "dynamics.h"

constexpr uint32_t kDiagWindowBuffers = 300;   // ~1.16 s bei 22.05 kHz

// Werte eines einzelnen Puffers (vom audioTask befüllt).
struct BufferStats {
    uint32_t elapsedUs;      // Rechenzeit (ohne Warten auf DAC und Mic)
    uint32_t waitUs;         // Warten auf freien DAC-Puffer
    uint32_t micWaitUs;      // Warten auf Mic-Samples
    uint32_t prepUs;         // Vorbereitung: Höhenanhebung, Erkennung, Carrier
    DualCoreTiming cores;
    float potNorm;
    float carrierHz;
    uint8_t wave;
    float formantShift;      // Formant-Verschiebung in Bändern
};

struct Diagnostics {
    // --- pro Sample -------------------------------------------------------
    inline void mic_sample(q16 x) {
        if (x < micMin) micMin = x;
        if (x > micMax) micMax = x;
    }
    inline void unvoiced_sample() { ++unvoicedSamples; }
    inline void gate_detector(q16 d) {
        if (d > gateDetMax) gateDetMax = d;
        gateDetSum += d;
    }

    // --- pro Puffer -------------------------------------------------------
    // Liest außerdem Kompressor/Gate/Mic/Filterbank aus und setzt deren
    // Diagnose-Zähler nach jeder Ausgabe zurück.
    void buffer_done(const BufferStats &s, Compressor &comp, const NoiseGate &gate);

private:
    void print_and_reset(Compressor &comp);

    uint32_t bufferCount = 0;
    uint64_t sumUs = 0;
    uint32_t maxUs = 0;
    uint64_t waitSumUs = 0;
    uint32_t waitMaxUs = 0;
    uint64_t micWaitSumUs = 0;
    uint64_t prepSumUs = 0;
    uint64_t core0BandsSumUs = 0;
    uint64_t core1ExtraWaitSumUs = 0;
    q16 micMin = kQ16One;
    q16 micMax = -kQ16One;
    uint32_t unvoicedSamples = 0;
    int64_t gateDetSum = 0;
    q16 gateDetMax = 0;
    // Momentaufnahmen des letzten Puffers im Fenster:
    int lastPotPermille = 0;
    int lastCarrierHz = 0;
    int lastCompEnvPermille = 0;
    int lastGateGainPermille = 0;
    int lastMicDcPermille = 0;
    uint32_t lastMicFill = 0;
    uint8_t lastWave = 0;
    float lastFormantShift = 0.0f;
};