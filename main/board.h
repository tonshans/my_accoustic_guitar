/*
 * board.h — central hardware configuration for the ESP32-S3 guitar FX unit.
 *
 * Signal chain (Phase 1, per ARCHITECTURE.md):
 *   Piezo -> MCP6002 (high-Z buffer) -> PCM1808 (ADC) -> I2S0 RX (GPIO4/5/6)
 *         -> [core 0 DSP task] -> I2S1 TX (GPIO15/16/17) -> MAX98357A -> exciter
 *
 * Local UI: SSD1306 OLED (I2C GPIO8/9), rotary encoder (GPIO10/11/12),
 * push button 1 (GPIO13: bypass/tuner), push button 2 (GPIO14: preset cycle).
 * Battery voltage monitor on GPIO2 (ADC1_CH1) through a resistor divider.
 * All pins are overridable through "Project Configuration" (Kconfig).
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#include "driver/i2s_std.h"
#include "driver/gpio.h"

/* ---------------- Audio geometry ---------------- */
#define SAMPLE_RATE          CONFIG_SAMPLE_RATE            /* 48000 Hz */
#define FRAME_SAMPLES        CONFIG_AUDIO_FRAME_SAMPLES    /* 128 samples/block = 2.66 ms */
#define BLOCK_PERIOD_US      (FRAME_SAMPLES * 1000000 / SAMPLE_RATE)  /* ~2666 us */
#define NUM_CHANNELS         2                             /* I2S std Philips, 32-bit slots */
#define AUDIO_BIT_SAMPLE     I2S_DATA_BIT_WIDTH_32BIT      /* PCM1808/MAX98357 in 32-bit slots */

/* Ring buffers between the I2S DMA callbacks and the DSP task:
 * 16 blocks => 16*128/48000 ~= 42.7 ms of decoupling capacity (absorbs
 * scheduling jitter; it does NOT add signal latency because blocks are
 * consumed as soon as they are ready). */
#define RING_BLOCKS          16
#define BLOCK_SAMPLES        FRAME_SAMPLES
#define BLOCK_WORDS          (BLOCK_SAMPLES * NUM_CHANNELS)          /* int32 words */
#define BYTES_PER_BLOCK      (BLOCK_WORDS * sizeof(int32_t))
#define RING_BYTES           (BYTES_PER_BLOCK * RING_BLOCKS)

/* 32-bit PCM samples arrive left-justified in the slot; >>11 gives Q15 headroom
 * so that mixing several effect paths never overflows int16 at the DAC stage. */
#define SAMPLE_SHIFT         11
#define Q15_MAX              0x7FFF

/* ---------------- I2S0 RX: PCM1808 (BCLK=4, LRCK=5, DATA=6) ---------------- */
#define I2S_IN_PORT          I2S_NUM_0
#define I2S_IN_BCK_IO        (gpio_num_t)CONFIG_I2S_IN_BCK_PIN
#define I2S_IN_WS_IO         (gpio_num_t)CONFIG_I2S_IN_WS_PIN
#define I2S_IN_DIN_IO        (gpio_num_t)CONFIG_I2S_IN_DIN_PIN
/* MCLK is not wired on this board; -1 maps to I2S_GPIO_UNUSED. */
#define I2S_IN_MCLK_IO       (CONFIG_I2S_IN_MCLK_PIN < 0 ? I2S_GPIO_UNUSED \
                                : (gpio_num_t)CONFIG_I2S_IN_MCLK_PIN)

/* ---------------- I2S1 TX: MAX98357A (BCLK=15, LRCK=16, DATA=17) ---------------- */
#define I2S_OUT_PORT         I2S_NUM_1
#define I2S_OUT_BCK_IO       (gpio_num_t)CONFIG_I2S_OUT_BCK_PIN
#define I2S_OUT_WS_IO        (gpio_num_t)CONFIG_I2S_OUT_WS_PIN
#define I2S_OUT_DOUT_IO      (gpio_num_t)CONFIG_I2S_OUT_DOUT_PIN
#define I2S_OUT_MCLK_IO      (CONFIG_I2S_OUT_MCLK_PIN < 0 ? I2S_GPIO_UNUSED \
                                : (gpio_num_t)CONFIG_I2S_OUT_MCLK_PIN)

/* ---------------- OLED (SSD1306, software I2C master bus) ---------------- */
#define OLED_SDA_PIN         (gpio_num_t)CONFIG_OLED_SDA_PIN
#define OLED_SCL_PIN         (gpio_num_t)CONFIG_OLED_SCL_PIN
#define OLED_I2C_ADDR        CONFIG_OLED_ADDR
#define OLED_I2C_PORT        ((i2c_port_t)CONFIG_OLED_I2C_PORT)

/* ---------------- Encoder & buttons ----------------
 * Rotary encoder: A=GPIO10 B=GPIO11 SW=GPIO12 -> page nav / param edit / select
 * BTN_ACT   GPIO13: short = bypass toggle, long (>700 ms) = tuner on/off
 * BTN_PRESET GPIO14: short = cycle preset, long = save preset to NVS      */
#define ENC_A_PIN            (gpio_num_t)CONFIG_ENC_A_PIN
#define ENC_B_PIN            (gpio_num_t)CONFIG_ENC_B_PIN
#define BTN_ENC_PIN          (gpio_num_t)CONFIG_BTN_ENC_PIN
#define BTN_ACT_PIN          (gpio_num_t)CONFIG_BTN_ACT_PIN
#define BTN_PRESET_PIN       (gpio_num_t)CONFIG_BTN_PRESET_PIN

#define LONG_PRESS_MS        700

/* ---------------- Battery monitor ---------------- */
#define BAT_ADC_PIN          GPIO_NUM_2          /* ADC1_CH1 */
#define BAT_ADC_CHANNEL      ADC_CHANNEL_1
#define BAT_DIVIDER_X100     CONFIG_BAT_DIVIDER_X100
#define BAT_FULL_MV          CONFIG_BAT_FULL_MV
#define BAT_EMPTY_MV         CONFIG_BAT_EMPTY_MV

/* ---------------- Task priorities & core binding (ARCHITECTURE.md §4) -------
 * Core 0 (highest): the whole audio domain — RX DMA feed, DSP pipeline,
 *                   TX drain, and the offline tuner analysis. Nothing on
 *                   core 0 may block, allocate, or touch flash.
 * Core 1 (normal):  UI rendering, encoder/button scan, NVS preset I/O,
 *                   health monitor.                                    */
#define PRIO_DSP             22      /* core 0: owns the block clock      */
#define PRIO_AUDIO           20      /* reserved (DMA cbs run in ISR ctx) */
#define PRIO_UI              5       /* core 1                            */
#define PRIO_MONITOR         2       /* core 1                            */
#define PRIO_MAIN            5

#define CORE_AUDIO           0
#define CORE_UI              1

#endif /* __BOARD_H__ */
