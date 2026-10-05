#pragma once
// =====================================================================
// controls.h - Pot, Taster, Status-LED
// =====================================================================
#include "FreeRTOS.h"
#include "queue.h"

// Queues zwischen audioTask und controlTask (je 1 Eintrag, Overwrite):
extern QueueHandle_t g_potRawQueue;      // float  audioTask -> control: Pot roh 0..1
extern QueueHandle_t g_carrierFreqQueue; // float  control -> audioTask: geglättete Tonhöhe (Hz)
extern QueueHandle_t g_waveQueue;        // uint8_t control -> audioTask: gewählte Wellenform

// In main() VOR dem Start der Tasks aufrufen.
void controls_create_queues();

// ADC für den Pot einrichten (gehört dem audioTask).
void setup_adc();

// Pot lesen, 0..1 (ggf. invertiert, siehe kInvertPot). Nur im audioTask.
float read_pot();

// FreeRTOS-Task: Pot glätten/mappen, Taster entprellen, LED-Blinkcode.
void controlTask(void *);