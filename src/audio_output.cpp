// =====================================================================
// audio_output.cpp - PCM5102A-DAC über pico-extras (pico_audio_i2s)
// =====================================================================
// Ausgelagert aus main.cpp, inhaltlich unverändert. Belegt pio0 und
// DMA-Kanal 0 - darum VOR setup_mic() aufrufen (das Mic holt sich pio1
// und einen freien DMA-Kanal). DAC-Pins: CMakeLists.txt.

#include "audio_output.h"
#include "config.h"
#include "pico/stdlib.h"

audio_buffer_pool_t *setup_audio() {
    static audio_format_t audioFormat = {
        .sample_freq = kSampleRateHz,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2,
    };
    static audio_buffer_format_t producerFormat = {
        .format = &audioFormat,
        .sample_stride = 4,
    };

    audio_buffer_pool_t *pool = audio_new_producer_pool(&producerFormat, 4, kBufferSamples);

    audio_i2s_config_t i2sConfig = {
        .data_pin = PICO_AUDIO_I2S_DATA_PIN,
        .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
        .dma_channel = 0,
        .pio_sm = 0,
    };

    const audio_format_t *outputFormat = audio_i2s_setup(&audioFormat, &i2sConfig);
    if (!outputFormat) {
        panic("I2S-Setup fehlgeschlagen - Pins/Format pruefen");
    }

    audio_i2s_connect(pool);
    audio_i2s_set_enabled(true);
    return pool;
}