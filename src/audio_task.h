#pragma once
// audio_task.h - Audio-Ablauf pro Puffer (FreeRTOS-Task, hohe Priorität)

// Initialisiert alle Module (DAC, Mic, Core1, ...) und läuft dann endlos:
// Mic lesen -> Vorbereitung (Höhenanhebung, Erkennung, Carrier) ->
// Filterbank auf beiden Kernen -> Kompressor/Gate -> DAC.
void audioTask(void *);