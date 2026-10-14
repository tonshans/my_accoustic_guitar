/*
 * board.h — central hardware configuration for the ESP32-S3 guitar FX unit.
 *
 * Signal chain (Phase 1):
 *   Piezo -> MCP6002 (high-Z buffer) -> PCM1808 (ADC) -> I2S0 RX
 *         -> [core 0 DSP task] -> I2S1 TX -> MAX98357A -> surface exciter
 *
 * Local UI: SSD1306 OLED (I2C), rotary encoder, 2 push buttons.
 * All pins are overridable through "Project Configuration" (Kconfig).
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#include "driver/i2s_std.h"
#include "driver/gpio.h"

/* ---------------- Audio geometry ---------------- */
#define SAMPLE_RATE          CONFIG_SAMPLE_RATE            /* 44100 Hz */
#define FRAME_SAMPLES        CONFIG_AUDIO_FRAME_SAMPLES    /* 128 samples/block */
#define NUM_CHANNELS         2                             /* I2S std Philips, 32-bit slots */
#define AUDIO_BIT_SAMPLE     I2S_DATA_BIT_WIDTH_32BIT      /* PCM1808/MAX98357 in 32-bit slots */

/* Ring buffer between the I2S DMA callbacks and the DSP task:
 * 16 blocks => 16*128/44100 ~= 46 ms of buffering (absorbs scheduling jitter,
 * does NOT add to signal latency because blocks are consumed as soon as ready). */
#define RING_BLOCKS          16
#define BLOCK_SAMPLES        FRAME_SAMPLES
#define BLOCK_WORDS          (BLOCK_SAMPLES * NUM_CHANNELS)          /* int32 words */
#define BYTES_PER_BLOCK      (BLOCK_WORDS * sizeof(int32_t))
#define RING_BYTES           (BYTES_PER_BLOCK * RING_BLOCKS)

/* 32-bit PCM samples arrive left-justified in the slot; >>11 gives Q15 headroom
 * so that mixing several effect paths never overflows int16 at the DAC stage. */
#define SAMPLE_SHIFT         11
#define Q15_MAX              0x7FFF

/* ---------------- I2S0 RX: PCM1808 ---------------- */
#define I2S_IN_PORT          I2S_NUM_0
#define I2S_IN_BCK_IO        (gpio_num_t)CONFIG_I2S_IN_BCK_PIN
#define I2S_IN_WS_IO         (gpio_num_t)CONFIG_I2S_IN_WS_PIN
#define I2S_IN_DIN_IO        (gpio_num_t)CONFIG_I2S_IN_DIN_PIN
#define I2S_IN_MCLK_IO       (gpio_num_t)CONFIG_I2S_IN_MCLK_PIN

/* ---------------- I2S1 TX: MAX98357A ---------------- */
#define I2S_OUT_PORT         I2S_NUM_1
#define I2S_OUT_BCK_IO       (gpio_num_t)CONFIG_I2S_OUT_BCK_PIN
#define I2S_OUT_WS_IO        (gpio_num_t)CONFIG_I2S_OUT_WS_PIN
#define I2S_OUT_DOUT_IO      (gpio_num_t)CONFIG_I2S_OUT_DOUT_PIN
#define I2S_OUT_MCLK_IO      (gpio_num_t)CONFIG_I2S_OUT_MCLK_IO

/* ---------------- OLED (SSD1306, software I2C master bus) ---------------- */
#define OLED_SDA_PIN         (gpio_num_t)CONFIG_OLED_SDA_PIN
#define OLED_SCL_PIN         (gpio_num_t)CONFIG_OLED_SCL_PIN
#define OLED_I2C_ADDR        CONFIG_OLED_ADDR
#define OLED_I2C_PORT        I2C_NUM_0

/* ---------------- Encoder & buttons ---------------- */
#define ENC_A_PIN            (gpio_num_t)CONFIG_ENC_A_PIN
#define ENC_B_PIN            (gpio_num_t)CONFIG_ENC_B_PIN
#define BTN_SEL_PIN          (gpio_num_t)CONFIG_BTN_SEL_PIN   /* short: next preset, long: save */
#define BTN_ACT_PIN          (gpio_num_t)CONFIG_BTN_ACT_PIN   /* short: bypass,   long: tuner  */

#define LONG_PRESS_MS        700

/* ---------------- Task priorities (strict hierarchy) ----------------
 * audio_rx (22) > audio_tx (21) > dsp (20)  — all on core 0.
 * ui/meter (5) and monitor (2) on core 1, so nothing ever preempts audio. */
#define PRIO_AUDIO_RX        22
#define PRIO_AUDIO_TX        21
#define PRIO_DSP             20
#define PRIO_UI              5
#define PRIO_MONITOR         2
#define PRIO_MAIN            5

#define CORE_AUDIO           0
#define CORE_UI              1

#endif /* __BOARD_H__ */
