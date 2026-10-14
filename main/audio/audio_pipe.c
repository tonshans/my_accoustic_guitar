/*
 * audio_pipe.c — dual I2S std-mode pipes with zero-copy-ish DMA callbacks.
 */
#include <string.h>
#include "audio_pipe.h"
#include "board.h"
#include "freertos/FreeRTOS.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2s_std.h"

static const char *TAG = "audio_pipe";

audio_pipe_t g_audio;

static i2s_chan_handle_t rx_chan;
static i2s_chan_handle_t tx_chan;

/* ------------------- RX callback (I2S ISR context, core 0, IRAM) --------- */
IRAM_ATTR static bool rx_dma_cb(i2s_chan_handle_t handle,
                                i2s_event_data_t *event, void *user_ctx)
{
    size_t n = gtr_ring_write(&g_audio.in_ring, event->data, event->size);
    if (n < event->size) {
        g_audio.rx_overruns++;
    }
    return false;
}

/* ------------------- TX callback (core 0, IRAM) ------------------- */
IRAM_ATTR static bool tx_dma_cb(i2s_chan_handle_t handle,
                                i2s_event_data_t *event, void *user_ctx)
{
    size_t want = event->size;
    memset(event->data, 0, want);                    /* default = silence */
    size_t n = gtr_ring_read_nb(&g_audio.out_ring, event->data, want);
    if (n < want) {
        g_audio.tx_underruns++;                      /* smoothed fade keeps it click-free */
    }
    return false;
}

/* ------------------- init ------------------- */
esp_err_t audio_pipe_init(void)
{
    ESP_RETURN_ON_ERROR(gtr_ring_init(&g_audio.in_ring),  __TAG, "in ring");
    ESP_RETURN_ON_ERROR(gtr_ring_init(&g_audio.out_ring), __TAG, "out ring");
    g_audio.rx_overruns = 0;
    g_audio.tx_underruns = 0;

    /* ---- I2S0: PCM1808 ADC (ESP32-S3 = master clock provider) ---- */
    i2s_chan_config_t rx_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    rx_cfg.auto_clear  = true;
    rx_cfg.dma_desc_num = 6;
    rx_cfg.dma_frame_num = FRAME_SAMPLES;      /* 1 DMA block == 1 DSP block */
    ESP_RETURN_ON_ERROR(i2s_new_channel(&rx_cfg, NULL, &rx_chan), __TAG, "rx chan");

    i2s_std_config_t rx_std = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(AUDIO_BIT_SAMPLE,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_IN_MCLK_IO,
            .bclk = I2S_IN_BCK_IO,
            .ws   = I2S_IN_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_IN_DIN_IO,
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx_chan, &rx_std), __TAG, "rx std");

    i2s_event_callbacks_t rx_cbs = { .on_recv = rx_dma_cb };
    ESP_RETURN_ON_ERROR(i2s_channel_register_event_callback(rx_chan, &rx_cbs, NULL),
                        __TAG, "rx cb");

    /* ---- I2S1: MAX98357A amp (no MCLK needed — internal PLL) ---- */
    i2s_chan_config_t tx_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    tx_cfg.auto_clear   = true;                 /* pad underruns with zeros */
    tx_cfg.dma_desc_num = 6;
    tx_cfg.dma_frame_num = FRAME_SAMPLES;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&tx_cfg, &tx_chan, NULL), __TAG, "tx chan");

    i2s_std_config_t tx_std = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(AUDIO_BIT_SAMPLE,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_OUT_MCLK_IO,            /* -1 => unused */
            .bclk = I2S_OUT_BCK_IO,
            .ws   = I2S_OUT_WS_IO,
            .dout = I2S_OUT_DOUT_PIN,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx_chan, &tx_std), __TAG, "tx std");

    i2s_event_callbacks_t tx_cbs = { .on_send = tx_dma_cb };
    ESP_RETURN_ON_ERROR(i2s_channel_register_event_callback(tx_chan, &tx_cbs, NULL),
                        __TAG, "tx cb");

    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx_chan), __TAG, "en rx");
    /* TX is enabled by main only AFTER the output ring has been pre-filled
     * with silence, so the MAX98357A never sees garbage at start-up
     * (requirement 5.3: no pops or clicks). */
    return ESP_OK;
}

void audio_pipe_enable_tx(void)
{
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
}

void audio_pipe_disable_tx(void)
{
    ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));
}

void audio_pipe_prefill_silence(int blocks)
{
    int32_t silence[BLOCK_WORDS];
    memset(silence, 0, sizeof(silence));
    for (int i = 0; i < blocks; i++) {
        gtr_ring_write(&g_audio.out_ring, silence, BYTES_PER_BLOCK);
    }
}
