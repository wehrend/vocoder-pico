// =====================================================================
// controls.cpp - Bedienelemente: Pot (Tonhöhe), Taster (Wellenform),
// Status-LED (Blinkcode)
// =====================================================================
// Ausgelagert aus main.cpp, inhaltlich unverändert.
//
// Aufteilung (siehe DEVLOG, Priority-Starvation-Fix / ADC-Sharing):
// - Der ADC gehört allein dem audioTask: er liest den Pot einmal pro
//   Puffer (read_pot) und schickt den Rohwert per Queue.
// - controlTask (niedrige Priorität) glättet und mappt den Pot-Wert,
//   wertet den Taster aus und steuert die LED.

#include "controls.h"
#include "config.h"
#include "carrier_wave.h"

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <cstdio>

QueueHandle_t g_potRawQueue = nullptr;
QueueHandle_t g_carrierFreqQueue = nullptr;
QueueHandle_t g_waveQueue = nullptr;

void controls_create_queues() {
    g_potRawQueue = xQueueCreate(1, sizeof(float));
    g_carrierFreqQueue = xQueueCreate(1, sizeof(float));
    g_waveQueue = xQueueCreate(1, sizeof(uint8_t));
    if (!g_potRawQueue || !g_carrierFreqQueue || !g_waveQueue) {
        panic("Queue-Erstellung fehlgeschlagen (Heap zu klein?)");
    }
}

void setup_adc() {
    adc_init();
    adc_gpio_init(POT_ADC_GPIO);
    // Einziger ADC-Kanal ist jetzt der Pot - einmal auswählen reicht.
    adc_select_input(POT_ADC_CHANNEL);
}

static float read_adc_normalized() {
    uint16_t raw = adc_read();
    return (float)raw / 4095.0f;
}

// Umkehr-Schalter für den Fall, dass das Poti physisch andersherum
// verkabelt ist als die Software erwartet (siehe DEVLOG - `pot=`
// blieb bei zwei Tests trotz Drehens am selben Anschlag auf ~1000
// hängen, noch nicht abschließend bestätigt, ob das an der
// Verkabelung lag oder schlicht zweimal derselbe Anschlag getroffen
// wurde). Auf true stellen, falls sich am tiefen Poti-Anschlag
// weiterhin `pot=` nahe 1000 statt nahe 0 zeigt - kein Kabel-Umbau
// nötig, einfach hier umschalten.
constexpr bool kInvertPot = false;

float read_pot() {
    float potNorm = read_adc_normalized();
    if (kInvertPot) potNorm = 1.0f - potNorm;
    return potNorm;
}

void controlTask(void *) {
    float smoothedHz = 220.0f;

    // Taster (GP14, Pullup -> gedrückt = 0) und Status-LED (GP25).
    gpio_init(kWaveButtonPin);
    gpio_set_dir(kWaveButtonPin, GPIO_IN);
    gpio_pull_up(kWaveButtonPin);
    gpio_init(kStatusLedPin);
    gpio_set_dir(kStatusLedPin, GPIO_OUT);

    uint8_t wave = (kFixedWave >= 0) ? (uint8_t)kFixedWave : (uint8_t)kWaveImpulse;
    xQueueOverwrite(g_waveQueue, &wave);

    // Entprellung: Zustand muss zwei Abfragen (~40 ms) stabil sein.
    bool stablePressed = false;
    bool lastRaw = false;
    // Blinkcode: (wave+1) kurze Blitze, dann Pause - zeitgesteuert über
    // den FreeRTOS-Tick, unabhängig von der Schleifendauer.
    constexpr TickType_t kBlinkOn    = pdMS_TO_TICKS(120);
    constexpr TickType_t kBlinkOff   = pdMS_TO_TICKS(180);
    constexpr TickType_t kBlinkPause = pdMS_TO_TICKS(900);
    TickType_t blinkPhaseStart = xTaskGetTickCount();
    int blinkStep = 0;   // 0..2*(wave+1)-1: an/aus im Wechsel, danach Pause

    for (;;) {
        float potNorm;
        if (xQueueReceive(g_potRawQueue, &potNorm, pdMS_TO_TICKS(50)) == pdTRUE) {
            float targetHz = kMinCarrierHz + potNorm * (kMaxCarrierHz - kMinCarrierHz);
            smoothedHz += (targetHz - smoothedHz) * 0.2f;
            xQueueOverwrite(g_carrierFreqQueue, &smoothedHz);
        }

        // --- Taster ---
        bool raw = !gpio_get(kWaveButtonPin);
        if (kFixedWave < 0 && raw == lastRaw && raw != stablePressed) {
            stablePressed = raw;
            if (stablePressed) {  // Flanke "gedrückt" -> nächste Wellenform
                wave = (uint8_t)((wave + 1) % kWaveCount);
                xQueueOverwrite(g_waveQueue, &wave);
                printf("Wellenform: %d/%d %s\n", wave + 1, (int)kWaveCount, kWaveNames[wave]);
                blinkStep = 0;    // Blinkcode sofort neu starten
                blinkPhaseStart = xTaskGetTickCount();
            }
        }
        lastRaw = raw;

        // --- Blinkcode ---
        int flashes = wave + 1;
        int steps = 2 * flashes;              // an, aus, an, aus, ...
        TickType_t now = xTaskGetTickCount();
        TickType_t dur = (blinkStep >= steps) ? kBlinkPause
                       : ((blinkStep % 2 == 0) ? kBlinkOn : kBlinkOff);
        if (now - blinkPhaseStart >= dur) {
            blinkPhaseStart = now;
            blinkStep = (blinkStep >= steps) ? 0 : blinkStep + 1;
        }
        gpio_put(kStatusLedPin, (blinkStep < steps) && (blinkStep % 2 == 0));

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}