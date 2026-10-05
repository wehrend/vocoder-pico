// =====================================================================
// main.cpp - Einstiegspunkt: 10-Band-Vocoder auf dem RP2040
// =====================================================================
// Raspberry Pi Pico + INMP441 (I2S-MEMS-Mic) + PCM5102A (I2S-DAC),
// FreeRTOS auf Core0, Core1 als eigene Schleife für die halbe Filterbank.
// Software-Referenz: modular-synth (VocoderBands.ts u.a.).
//
// Module (src/):
//   config.h            Pins, Samplerate, Bänder, Tonhöhenbereich
//   audio_task.*        Ablauf pro Puffer (Mic -> Vorbereitung ->
//                       Filterbank -> Dynamik -> DAC)
//   mic_input.*         INMP441 über PIO (i2s_mic.pio) + DMA-Ringpuffer
//   audio_output.*      PCM5102A über pico-extras
//   voiced_unvoiced.*   Rumpelfilter, Höhenanhebung, Stimmhaft/Stimmlos
//   carrier.*           Carrier-Wellenformen mit Pegelabgleich
//   carrier_wave.h      Wellenform-Auswahl (+ Kalibrier-Schalter)
//   filterbank.*        10 Bänder: Analyse, Synthese, Bandgewichtung
//   dual_core.*         Verteilung der Filterbank auf beide Kerne
//   dynamics.*          Kompressor, Noise-Gate
//   controls.*          Pot, Taster, Status-LED (controlTask)
//   diagnostics.*       Messwerte und Diagnoseausgabe (USB-Seriell)
//   fixed_point.h, biquad_fixed.h, vocoder_band_fixed.h
//                       Festkomma-Grundlagen (Q16 / Q8.24)
//
// Entstehung, Messungen und Begründungen: DEVLOG und git-Historie.

#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

#include "audio_task.h"
#include "controls.h"

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    panic("Stack-Overflow in Task: %s", pcTaskName);
}

extern "C" void vApplicationMallocFailedHook(void) {
    panic("FreeRTOS malloc fehlgeschlagen - configTOTAL_HEAP_SIZE in FreeRTOSConfig.h zu klein?");
}

int main() {
    stdio_init_all();

    controls_create_queues();

    // audioTask (Core0, hohe Priorität) verteilt die Bandberechnung pro
    // Puffer auf beide Kerne: Core1 läuft außerhalb von FreeRTOS
    // (dual_core.cpp) und wird aus audioTask heraus gestartet.
    xTaskCreate(audioTask, "audio", 1024, nullptr, /*priority=*/3, nullptr);
    xTaskCreate(controlTask, "control", 512, nullptr, /*priority=*/1, nullptr);

    vTaskStartScheduler();

    panic("vTaskStartScheduler() zurueckgekehrt - Heap zu klein?");
    return 0;
}