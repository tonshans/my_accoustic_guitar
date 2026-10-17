/*
 * presets.c — NVS persistence for the 4 preset slots.
 * Each slot is stored as a fixed-size blob ("p0".."p3") = magic + version +
 * dsp_settings_t. A corrupt/absent/old blob falls back to Kconfig defaults,
 * so firmware updates can never brick the audio path.
 */
#include "presets.h"
#include "board.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "presets";
#define PRESET_MAGIC 0x47545231u   /* 'G''T''R''1' */
#define PRESET_VERSION 1

static int s_current;

typedef struct {
    uint32_t magic;
    uint32_t version;
    dsp_settings_t s;
} blob_t;

static void defaults(dsp_settings_t *s)
{
    memset(s, 0, sizeof(*s));
    s->preset           = 0;
    s->fx_on            = true;
    s->delay_mix        = CONFIG_DELAY_MIX_PCT_DFLT;
    s->chorus_mix       = CONFIG_CHORUS_MIX_PCT_DFLT;
    s->reverb_mix       = CONFIG_REVERB_MIX_PCT_DFLT;
    s->delay_ms         = CONFIG_DELAY_TIME_MS_DFLT;
    s->delay_fb         = CONFIG_DELAY_FEEDBACK_PCT_DFLT;
    s->chorus_rate_mhz  = CONFIG_CHORUS_RATE_MHZ;
    s->chorus_depth_us  = CONFIG_CHORUS_DEPTH_US;
    s->hpf_hz           = (uint8_t)(CONFIG_HPF_CUTOFF_HZ * 10); /* kHz*10 enc. */
    s->master_pct       = CONFIG_OUTPUT_GAIN_PCT;
}

int presets_load(int idx, dsp_settings_t *out)
{
    if (idx < 0 || idx >= NUM_PRESETS) idx = 0;
    char key[4];
    snprintf(key, sizeof(key), "p%d", idx);

    nvs_handle_t h;
    blob_t b;
    size_t sz = sizeof(b);
    if (nvs_open("gtr", NVS_READONLY, &h) == ESP_OK) {
        esp_err_t e = nvs_get_blob(h, key, &b, &sz);
        nvs_close(h);
        if (e == ESP_OK && sz == sizeof(b) &&
            b.magic == PRESET_MAGIC && b.version == PRESET_VERSION) {
            *out = b.s;
            out->preset = (int8_t)idx;
            s_current = idx;
            dsp_apply_settings(out);
            ESP_LOGI(TAG, "loaded preset %d from NVS", idx);
            return idx;
        }
    }
    defaults(out);
    out->preset = (int8_t)idx;
    s_current = idx;
    dsp_apply_settings(out);
    return idx;
}

void presets_save(int idx, const dsp_settings_t *s)
{
    if (idx < 0 || idx >= NUM_PRESETS) return;
    blob_t b = { .magic = PRESET_MAGIC, .version = PRESET_VERSION, .s = *s };
    b.s.preset = (int8_t)idx;
    char key[4];
    snprintf(key, sizeof(key), "p%d", idx);

    nvs_handle_t h;
    esp_err_t e = nvs_open("gtr", NVS_READWRITE, &h);
    if (e == ESP_OK) {
        e = nvs_set_blob(h, key, &b, sizeof(b));
        if (e == ESP_OK) e = nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "save preset %d: %s", idx, esp_err_to_name(e));
}

void presets_cycle_next(void)
{
    /* Per requirement "preset cycle button": cycling does NOT auto-save the
     * edited outgoing slot — that would surprise the player. Use the encoder
     * long press (or BTN_PRESET long press) explicitly to save. */
    dsp_settings_t cur;
    presets_load((s_current + 1) % NUM_PRESETS, &cur);
}

void presets_cycle_prev(void)
{
    s_current = (s_current + NUM_PRESETS - 1) % NUM_PRESETS;
    dsp_settings_t cur;
    presets_load(s_current, &cur);
}

void presets_save_current(void)
{
    dsp_settings_t cur;
    dsp_get_settings(&cur);
    presets_save(s_current, &cur);
}

int presets_current(void) { return s_current; }
