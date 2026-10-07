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
#include "filterbank.h"
#include "display.h"

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <cstdio>
#include <cmath>

QueueHandle_t g_potRawQueue = nullptr;
QueueHandle_t g_carrierFreqQueue = nullptr;
QueueHandle_t g_waveQueue = nullptr;
QueueHandle_t g_formantRawQueue = nullptr;
QueueHandle_t g_formantShiftQueue = nullptr;

// Formant-Pot: Mitte = keine Verschiebung. In der Mittelzone (±kFormant-
// DeadZone des Drehwegs) ist die Verschiebung exakt 0 - dann ist der
// Klang bitgenau wie ohne Formant-Funktion. Außerhalb linear bis
// ±kFormantMaxShiftBands an den Anschlägen.
constexpr float kFormantDeadZone = 0.06f;
constexpr bool  kInvertFormantPot = false;   // falls das Poti andersherum verdrahtet ist

static float formant_pot_to_shift(float potNorm) {
    float c = 2.0f * potNorm - 1.0f;                 // -1 .. +1, Mitte = 0
    float a = (c < 0.0f) ? -c : c;
    if (a < kFormantDeadZone) return 0.0f;
    float scaled = (a - kFormantDeadZone) / (1.0f - kFormantDeadZone);
    if (scaled > 1.0f) scaled = 1.0f;
    return ((c < 0.0f) ? -scaled : scaled) * kFormantMaxShiftBands;
}

void controls_create_queues() {
    g_potRawQueue = xQueueCreate(1, sizeof(float));
    g_carrierFreqQueue = xQueueCreate(1, sizeof(float));
    g_waveQueue = xQueueCreate(1, sizeof(uint8_t));
    g_formantRawQueue = xQueueCreate(1, sizeof(float));
    g_formantShiftQueue = xQueueCreate(1, sizeof(float));
    if (!g_potRawQueue || !g_carrierFreqQueue || !g_waveQueue ||
        !g_formantRawQueue || !g_formantShiftQueue) {
        panic("Queue-Erstellung fehlgeschlagen (Heap zu klein?)");
    }
}

void setup_adc() {
    adc_init();
    adc_gpio_init(FORMANT_ADC_GPIO);
    if (kPitchPotEnabled) adc_gpio_init(POT_ADC_GPIO);
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
    adc_select_input(POT_ADC_CHANNEL);   // zwei Pots -> Kanal vor jedem Lesen wählen
    float potNorm = read_adc_normalized();
    if (kInvertPot) potNorm = 1.0f - potNorm;
    return potNorm;
}

float read_formant_pot() {
    adc_select_input(FORMANT_ADC_CHANNEL);
    float v = read_adc_normalized();
    return kInvertFormantPot ? 1.0f - v : v;
}

void controlTask(void *) {
    float smoothedHz = 220.0f;
    float smoothedFormantPot = 0.5f;   // Mitte = keine Verschiebung

    // Taster (GP14, Pullup -> gedrückt = 0) und Status-LED (GP25).
    gpio_init(kWaveButtonPin);
    gpio_set_dir(kWaveButtonPin, GPIO_IN);
    gpio_pull_up(kWaveButtonPin);
    display_init();
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

    // Anzeige: normalerweise die Formant-Verschiebung; nach einem Taster-
    // druck (und beim Start) für kWaveOverlay die gewählte Wellenform.
    // Aktualisiert wird nur bei Änderungen.
    constexpr TickType_t kWaveOverlay = pdMS_TO_TICKS(1500);
    float currentShift = 0.0f;
    int shownShiftCenti = -100000;          // erzwingt erste Anzeige
    bool overlayActive = true;
    TickType_t overlayStart = xTaskGetTickCount();
    display_show_wave(wave);

    for (;;) {
        float potNorm;
        if (xQueueReceive(g_potRawQueue, &potNorm, pdMS_TO_TICKS(50)) == pdTRUE) {
            float targetHz = kMinCarrierHz + potNorm * (kMaxCarrierHz - kMinCarrierHz);
            smoothedHz += (targetHz - smoothedHz) * 0.2f;
            xQueueOverwrite(g_carrierFreqQueue, &smoothedHz);
        }

        // --- Formant-Pot: glätten, auf Bänder abbilden ---
        float formantRaw;
        if (xQueueReceive(g_formantRawQueue, &formantRaw, 0) == pdTRUE) {
            smoothedFormantPot += (formantRaw - smoothedFormantPot) * 0.2f;
            float shift = formant_pot_to_shift(smoothedFormantPot);
            currentShift = shift;
            xQueueOverwrite(g_formantShiftQueue, &shift);
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
                display_show_wave(wave);
                overlayActive = true;
                overlayStart = xTaskGetTickCount();
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

        // --- Anzeige ---
        if (overlayActive && (xTaskGetTickCount() - overlayStart >= kWaveOverlay)) {
            overlayActive = false;
            shownShiftCenti = -100000;      // Formant sofort wieder anzeigen
        }
        if (!overlayActive) {
            int centi = (int)lroundf(currentShift * 100.0f);
            if (centi != shownShiftCenti) {
                display_show_formant(currentShift);
                shownShiftCenti = centi;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}