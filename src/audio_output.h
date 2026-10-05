#pragma once
// audio_output.h - PCM5102A-DAC (I2S-Ausgang über pico-extras)
#include "pico/audio_i2s.h"

// Richtet I2S-Ausgabe und Pufferpool ein (4 Puffer à kBufferSamples,
// Stereo, 16 Bit) und startet die Ausgabe. VOR setup_mic() aufrufen.
audio_buffer_pool_t *setup_audio();