// =====================================================================
// diagnostics.cpp - Ausgabe der Diagnosezeilen
// =====================================================================
// Ausgelagert aus main.cpp; Format der Ausgabe unverändert.
//
// Zeitstempel: build_stamp.h wird von CMake bei JEDEM Build neu erzeugt
// (cmake/build_stamp.cmake). __DATE__/__TIME__ zeigten nur die
// Kompilierzeit dieser einen Datei - seit der Aufteilung auf mehrere
// Dateien wäre der Stempel bei Änderungen anderswo stehen geblieben.

#include "diagnostics.h"
#include "config.h"
#include "mic_input.h"
#include "filterbank.h"
#include "carrier_wave.h"
#include "build_stamp.h"

#include <cmath>
#include <cstdio>

void Diagnostics::buffer_done(const BufferStats &s, Compressor &comp, const NoiseGate &gate) {
    waitSumUs += s.waitUs;
    if (s.waitUs > waitMaxUs) waitMaxUs = s.waitUs;
    micWaitSumUs += s.micWaitUs;
    prepSumUs += s.prepUs;
    core0BandsSumUs += s.cores.core0BandsUs;
    core1ExtraWaitSumUs += s.cores.core1ExtraWaitUs;
    sumUs += s.elapsedUs;
    if (s.elapsedUs > maxUs) maxUs = s.elapsedUs;

    lastPotPermille = (int)(s.potNorm * 1000.0f);
    lastCarrierHz = (int)s.carrierHz;
    lastWave = s.wave;
    lastCompEnvPermille = (int)(((int64_t)comp.envelope * 1000) / kQ16One);
    lastGateGainPermille = (int)(((int64_t)gate.gain * 1000) / kQ16One);
    lastMicDcPermille = (int)(((int64_t)mic_dc_level() * 1000) / kQ16One);
    lastMicFill = mic_fill();

    if (++bufferCount >= kDiagWindowBuffers) {
        print_and_reset(comp);
    }
}

void Diagnostics::print_and_reset(Compressor &comp) {
    constexpr uint32_t kBufferBudgetUs = (kBufferSamples * 1000000ull) / kSampleRateHz;
    const int64_t kSamplesInWindow = (int64_t)bufferCount * kBufferSamples;
    int micMinPermille = (int)(((int64_t)micMin * 1000) / kQ16One);
    int micMaxPermille = (int)(((int64_t)micMax * 1000) / kQ16One);

    printf("Diagnose[%s]: avg=%luus max=%luus wait=%luus waitMax=%luus budget=%luus pot=%d/1000 carrierHz=%d micMin=%d/1000 micMax=%d/1000 micDc=%d/1000 compEnv=%d/1000 gateGain=%d/1000 (100 Puffer)\n",
           kBuildStamp,
           (unsigned long)(sumUs / bufferCount), (unsigned long)maxUs,
           (unsigned long)(waitSumUs / bufferCount), (unsigned long)waitMaxUs,
           (unsigned long)kBufferBudgetUs,
           lastPotPermille, lastCarrierHz, micMinPermille, micMaxPermille,
           lastMicDcPermille, lastCompEnvPermille, lastGateGainPermille);
    printf("  mic: fill=%lu/%lu under=%lu over=%lu micWait=%luus\n",
           (unsigned long)lastMicFill, (unsigned long)kMicTargetFill,
           (unsigned long)mic_underruns(), (unsigned long)mic_overruns(),
           (unsigned long)(micWaitSumUs / bufferCount));
    printf("  carrier: %s\n", kWaveNames[lastWave]);
    printf("  vuv: stimmlos=%lu%% prep=%luus\n",
           (unsigned long)((uint64_t)unvoicedSamples * 100 / ((uint64_t)bufferCount * kBufferSamples)),
           (unsigned long)(prepSumUs / bufferCount));
    printf("  cores: core0Bands=%luus core1Warten=%luus (Core1 rechnet Bänder %d..%d)\n",
           (unsigned long)(core0BandsSumUs / bufferCount),
           (unsigned long)(core1ExtraWaitSumUs / bufferCount),
           kCore1FirstBand, kNumBands - 1);
    {
        float compMinGain = (float)comp.minGainSeen / (float)kQ16One;
        int compMinDb = (compMinGain > 0.0f) ? (int)(20.0f * log10f(compMinGain)) : -999;
        printf("  gateDet: avg=%d max=%d (x/10000, open=%d close=%d)  comp: maxReduktion=%ddB\n",
               (int)((gateDetSum * 10000) / ((int64_t)kQ16One * kSamplesInWindow)),
               (int)(((int64_t)gateDetMax * 10000) / kQ16One),
               (int)(kGateOpenThreshold * 10000.0f), (int)(kGateCloseThreshold * 10000.0f),
               compMinDb);
    }
    printf("  bands[0..%d]=", kNumBands - 1);
    for (int b = 0; b < kNumBands; ++b) {
        printf("%d ", (int)(((int64_t)g_filterbankDiag.bandMax[b] * 1000) / kQ16One));
    }
    printf("\n");
    // Mittelwerte: outAvg in 1/1000, bandsAvg in 1/10000 (feiner, sonst
    // liegen leise Bänder alle bei 0-5).
    printf("  outAvg[0..%d] (x/1000)=", kNumBands - 1);
    for (int b = 0; b < kNumBands; ++b) {
        printf("%d ", (int)((g_filterbankDiag.outSum[b] * 1000) / ((int64_t)kQ16One * kSamplesInWindow)));
    }
    printf("\n");
    printf("  bandsAvg[0..%d] (x/10000)=", kNumBands - 1);
    for (int b = 0; b < kNumBands; ++b) {
        printf("%d ", (int)((g_filterbankDiag.bandSum[b] * 10000) / ((int64_t)kQ16One * kSamplesInWindow)));
    }
    printf("\n");

    bufferCount = 0;
    sumUs = 0;
    maxUs = 0;
    waitSumUs = 0;
    waitMaxUs = 0;
    micWaitSumUs = 0;
    prepSumUs = 0;
    core0BandsSumUs = 0;
    core1ExtraWaitSumUs = 0;
    micMin = kQ16One;
    micMax = -kQ16One;
    unvoicedSamples = 0;
    gateDetSum = 0;
    gateDetMax = 0;
    g_filterbankDiag.reset();
    comp.minGainSeen = kQ16One;
}