/*
 * main.c — Phase 1 boot sequence + core-1 health monitor.
 *
 * Bring-up order (ARCHITECTURE.md §4/§6, PRD milestone 1):
 *   1. NVS (presets live in flash; failure is non-fatal -> Kconfig defaults)
 *   2. battery ADC (core 1 only)
 *   3. audio_pipe_init(): I2S0 RX (PCM1808) + I2S1 TX (MAX98357A), RX enabled,
 *      TX still OFF so the amp can never see garbage at start-up.
 *   4. dsp_init() + first preset load + dsp_apply_settings() so the pipeline
 *      starts with valid coefficients and gains.
 *   5. dsp_start_task(): the core-0 block-cadence task begins draining the RX
 *      ring immediately (pass-through path verified before effects).
 *   6. prefill the TX ring with silence, then enable TX -> pop-free start.
 *   7. ui_start_task() (OLED pages, encoder/buttons) + monitor task.
 *
 * The app main itself runs on core 1 (PRIO_MAIN); it deletes itself when the
 * bring-up completes so it consumes no runtime resources afterwards.
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"

#include "board.h"
#include "audio_pipe.h"
#include "dsp.h"
#include "tuner.h"
#include "presets.h"
#include "battery.h"
#include "ui.h"

static const char *TAG = "main";

/* DSP timing stats exported by dsp.c (display-only, lock-free reads). */
extern volatile uint32_t g_dsp_us_last, g_dsp_us_max, g_dsp_us_avg;
extern volatile uint32_t g_dsp_overruns;

/* ------------------------- health monitor (core 1) -------------------------
 * ARCHITECTURE.md §8: DMA underruns/overruns and DSP deadline misses are
 * logged and surfaced on the OLED SYS page — they must NEVER halt audio.
 * Also acts as the watchdog for the core-0 block clock: if g_dsp_beat stops
 * advancing we shout about it (recovery strategy is deliberately loud-free:
 * the TX callback keeps padding silence, so a stall degrades to mute, not
 * noise). */
static void monitor_task(void *arg)
{
    uint32_t last_beat = 0;
    uint32_t stalled_ticks = 0;
    uint32_t rx_o = 0, tx_u = 0, dsp_or = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t beat = g_dsp_beat;
        uint32_t new_rx = g_audio.rx_overruns - rx_o;   rx_o = g_audio.rx_overruns;
        uint32_t new_tx = g_audio.tx_underruns - tx_u;  tx_u = g_audio.tx_underruns;
        uint32_t new_dsp = g_dsp_overruns - dsp_or;     dsp_or = g_dsp_overruns;

        if (new_rx || new_tx || new_dsp) {
            ESP_LOGW(TAG, "audio health: +%lu rx-overrun +%lu tx-underrun +%lu dsp-deadline "
                          "(tot %lu/%lu/%lu) | dsp us last=%lu max=%lu avg=%lu",
                     (unsigned long)new_rx, (unsigned long)new_tx, (unsigned long)new_dsp,
                     (unsigned long)rx_o, (unsigned long)tx_u, (unsigned long)dsp_or,
                     (unsigned long)g_dsp_us_last, (unsigned long)g_dsp_us_max,
                     (unsigned long)g_dsp_us_avg);
        } else {
            ESP_LOGI(TAG, "OK %lu blocks/s | dsp %luus avg %luus max | bat %umV (%u%%)",
                     (unsigned long)(beat - last_beat),
                     (unsigned long)g_dsp_us_avg, (unsigned long)g_dsp_us_max,
                     battery_mv(), battery_pct());
        }

        /* Block-clock stall detection (~1 missed second of blocks). */
        if (beat == last_beat) {
            if (++stalled_ticks == 3)
                ESP_LOGE(TAG, "DSP block clock STALLED — audio output is muted silence");
        } else {
            stalled_ticks = 0;
            last_beat = beat;
            dsp_reset_timing_stats();      /* per-second max window */
        }
    }
}

/* ------------------------------- app main ------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "Guitar FX phase 1 boot (%d Hz, %d-sample blocks, period %d us)",
             SAMPLE_RATE, FRAME_SAMPLES, BLOCK_PERIOD_US);

    /* 1. NVS ------------------------------------------------------------- */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated — erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK)
        ESP_LOGW(TAG, "NVS unavailable (%s) — presets fall back to defaults",
                 esp_err_to_name(err));

    /* 2. Battery monitor (core 1 ADC; safe even before UI needs it) ------- */
    battery_init();

    /* 3. Audio plumbing: rings + both I2S channels, RX running ------------- */
    ESP_ERROR_CHECK(audio_pipe_init());

    /* 4. DSP state + first preset BEFORE any block is processed ------------ */
    dsp_init();
    dsp_settings_t s;
    int idx = presets_load(presets_current(), &s);
    dsp_apply_settings(&s);
    dsp_set_preset(idx);
    ESP_LOGI(TAG, "preset %d loaded (fx=%d hpf=%d master=%d%%)",
             idx + 1, (int)s.fx_on, s.hpf_hz, s.master_pct);

    /* 5. Start the core-0 audio domain ------------------------------------- */
    dsp_start_task();

    /* 6. Silence-prefill then enable TX: MAX98357A wakes into zeros, and the
     *    DSP task additionally ramps its first 4 blocks (fade-in). ---------- */
    audio_pipe_prefill_silence(4);
    audio_pipe_enable_tx();
    ESP_LOGI(TAG, "pass-through live: PCM1808 -> DSP -> MAX98357A");

    /* 7. Local interface + health monitor (both core 1) --------------------- */
    ui_start_task();
    xTaskCreatePinnedToCore(monitor_task, "monitor", 3072, NULL,
                            PRIO_MONITOR, NULL, CORE_UI);

    ESP_LOGI(TAG, "boot complete; free int RAM %u B, PSRAM %u B",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    vTaskDelete(NULL);   /* app_main runs on core 1; hand over to ui/monitor */
}
