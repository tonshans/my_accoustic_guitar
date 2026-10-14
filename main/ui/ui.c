/*
 * ui.c — OLED page manager + control mapping (core 1, UI task).
 *
 * Pages (encoder SW short cycles them):
 *   HOME    : preset / FX state / input level bar / battery
 *   MIX     : delay / chorus / reverb wet mixes (edit with A/B)
 *   DELAY   : time ms, feedback %
 *   CHORUS  : LFO rate Hz, depth us
 *   REVERB  : wet mix
 *   TUNE    : note name, cents needle, frequency, input peak
 *   SYS     : HPF cutoff, master gain, DSP timing, DMA error counters
 *
 * Buttons (per PRD 5.5):
 *   BTN_ACT short   -> bypass toggle (click-free crossfade in the DSP)
 *   BTN_ACT long    -> tuner on/off
 *   BTN_PRESET short-> cycle preset slot (loads from NVS)
 *   BTN_PRESET long -> save current settings into the active slot
 * Encoder: CW/CCW edits the selected row; SW short = next page,
 *          SW long = enter/exit edit mode for the current row.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ui.h"
#include "ssd1306.h"
#include "encoder.h"
#include "battery.h"
#include "board.h"
#include "dsp.h"
#include "tuner.h"
#include "presets.h"
#include "audio_pipe.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ui";

extern volatile uint32_t g_dsp_us_last, g_dsp_us_max, g_dsp_us_avg;
extern volatile uint32_t g_dsp_overruns;

/* --------------------------- model --------------------------- */
typedef struct {
    int field;         /* dsp field id or pseudo-field            */
    int min, max, step;/* editor bounds                             */
    const char *name;  /* left column label                         */
    int (*get)(void);
    void (*set)(int v);
} row_t;

/* pseudo fields handled outside dsp_set_field(): */
enum { PF_NONE = 100, PF_HPF_SEL };

static int get_delay_mix(void)   { dsp_settings_t s; dsp_get_settings(&s); return s.delay_mix; }
static int get_chorus_mix(void)  { dsp_settings_t s; dsp_get_settings(&s); return s.chorus_mix; }
static int get_reverb_mix(void)  { dsp_settings_t s; dsp_get_settings(&s); return s.reverb_mix; }
static int get_delay_ms(void)    { dsp_settings_t s; dsp_get_settings(&s); return s.delay_ms; }
static int get_delay_fb(void)    { dsp_settings_t s; dsp_get_settings(&s); return s.delay_fb; }
static int get_ch_rate(void)     { dsp_settings_t s; dsp_get_settings(&s); return s.chorus_rate_mhz; }
static int get_ch_depth(void)    { dsp_settings_t s; dsp_get_settings(&s); return s.chorus_depth_us; }
static int get_master(void)      { dsp_settings_t s; dsp_get_settings(&s); return s.master_pct; }
static int get_hpf_sel(void)     { dsp_settings_t s; dsp_get_settings(&s); return s.hpf_hz ? s.hpf_hz / 10 - 8 : 0; }
/* hpf_sel index -> cutoff: 0=OFF, 1=80 Hz, 2=90 Hz, 3=100 Hz */
static void set_hpf_sel(int i)   { static const int t[4] = {0, 80, 90, 100};
                                   dsp_set_field(FLD_HPF, t[i & 3]); }

/* per-page editable rows; row 0 of each page is the default selection */
static const row_t pg_mix[] = {
    { FLD_DELAY_MIX,   0, 100,  5, "DELAY WET", get_delay_mix,  NULL },
    { FLD_CHORUS_MIX,  0, 100,  5, "CHORUS WET", get_chorus_mix, NULL },
    { FLD_REVERB_MIX,  0, 100,  5, "REVERB WET", get_reverb_mix, NULL },
};
static const row_t pg_delay[] = {
    { FLD_DELAY_MS,   20, 400,  5, "TIME MS", get_delay_ms,  NULL },
    { FLD_DELAY_FB,    0,  90,  5, "FB %",    get_delay_fb,  NULL },
};
static const row_t pg_chorus[] = {
    { FLD_CHORUS_RATE,  100, 2000, 50, "RATE mHz", get_ch_rate,  NULL },
    { FLD_CHORUS_DEPTH,    0, 2000, 50, "DEPTH us", get_ch_depth, NULL },
};
static const row_t pg_reverb[] = {
    { FLD_REVERB_MIX,     0, 100,  5, "WET %",   get_reverb_mix, NULL },
};
static const row_t pg_sys[] = {
    { PF_HPF_SEL,     0, 3, 1, "HPF",     get_hpf_sel,  set_hpf_sel },
    { FLD_MASTER,     0, 100, 5, "MASTER", get_master,   NULL },
};

typedef enum { PAGE_HOME, PAGE_MIX, PAGE_DELAY, PAGE_CHORUS, PAGE_REVERB,
               PAGE_TUNE, PAGE_SYS, PAGE_COUNT } page_t;

static page_t s_page = PAGE_HOME;
static int    s_row;                 /* selected editable row        */
static bool   s_edit;                /* true = encoder edits param   */

static const char *page_title(page_t p)
{
    switch (p) {
    case PAGE_HOME:   return "GTR FX";
    case PAGE_MIX:    return "MIXER";
    case PAGE_DELAY:  return "DELAY";
    case PAGE_CHORUS: return "CHORUS";
    case PAGE_REVERB: return "REVERB";
    case PAGE_TUNE:   return "TUNER";
    default:          return "SYSTEM";
    }
}

static int page_rows(page_t p)
{
    switch (p) {
    case PAGE_MIX:    return (int)(sizeof(pg_mix)/sizeof(row_t));
    case PAGE_DELAY:  return (int)(sizeof(pg_delay)/sizeof(row_t));
    case PAGE_CHORUS: return (int)(sizeof(pg_chorus)/sizeof(row_t));
    case PAGE_REVERB: return (int)(sizeof(pg_reverb)/sizeof(row_t));
    case PAGE_SYS:    return (int)(sizeof(pg_sys)/sizeof(row_t));
    default:          return 0;
    }
}
static const row_t *page_table(page_t p)
{
    switch (p) {
    case PAGE_MIX:    return pg_mix;
    case PAGE_DELAY:  return pg_delay;
    case PAGE_CHORUS: return pg_chorus;
    case PAGE_REVERB: return pg_reverb;
    case PAGE_SYS:    return pg_sys;
    default:          return NULL;
    }
}

/* --------------------------- rendering --------------------------- */
static void draw_header(const char *title)
{
    oled_fill_rect(0, 0, 128, 9, true);           /* inverted banner   */
    oled_str(2, 0, title, true);                  /* inv=true -> white */
}

static void fmt_val(char *b, size_t n, const row_t *r, int v)
{
    switch (r->field) {
    case FLD_DELAY_MIX: case FLD_CHORUS_MIX: case FLD_REVERB_MIX:
    case FLD_DELAY_FB:  case FLD_MASTER:
        snprintf(b, n, "%d%%", v); break;
    case FLD_DELAY_MS:      snprintf(b, n, "%dms", v); break;
    case FLD_CHORUS_RATE:   snprintf(b, n, "%d.%02dHz", v/1000, (v%1000)/10); break;
    case FLD_CHORUS_DEPTH:  snprintf(b, n, "%uus", v); break;
    case PF_HPF_SEL:        snprintf(b, n, v ? "%dHz" : "OFF", v * 10 + 70); break;
    default:                snprintf(b, n, "%d", v); break;
    }
}

static void row_set(const row_t *r, int v)
{
    if (r->set) r->set(v);
    else        dsp_set_field(r->field, v);
}

static void render_pages(bool fx_on)
{
    char b[20];
    switch (s_page) {
    case PAGE_HOME: {
        draw_header("GTR FX");        dsp_settings_t s; dsp_get_settings(&s);
        snprintf(b, sizeof b, "PRESET %d  %s", presets_current() + 1,
                 fx_on ? "FX ON " : "BYPASS");
        oled_str(2, 1, b, false);
        snprintf(b, sizeof b, "DLY%u CHR%u VRB%u",
                 s.delay_mix, s.chorus_mix, s.reverb_mix);
        oled_str(2, 2, b, false);
        /* input level bar */
        int pk = dsp_peak_in();                   /* Q15 */
        int w = pk * 124 / 32767;
        oled_bar(2, 3, 124, w, true);
        snprintf(b, sizeof b, "BAT %u.%02uV %u%%%s",
                 battery_mv() / 1000, (battery_mv() % 1000) / 10,
                 battery_pct(), battery_low() ? "!" : " ");
        oled_str(2, 4, b, false);
        oled_str(2, 6, tuner_is_enabled() ? "TUNER ON (hold B1)"
                                          : "SW=nxt SEL=BYP TNR", false);
        break;
    }
    case PAGE_MIX: case PAGE_DELAY: case PAGE_CHORUS:
    case PAGE_REVERB: case PAGE_SYS: {
        const row_t *tbl = page_table(s_page);
        int nr = page_rows(s_page);
        draw_header(page_title(s_page));
        for (int i = 0; i < nr; i++) {
            int v = tbl[i].get();
            char val[16];
            fmt_val(val, sizeof val, &tbl[i], v);
            snprintf(b, sizeof b, "%c%-10s%s",
                     (i == s_row) ? (s_edit ? '>' : '*') : ' ',
                     tbl[i].name, val);
            oled_str(2, 1 + i * 2, b, false);
            if (i == s_row && tbl[i].min != tbl[i].max) {
                int span = tbl[i].max - tbl[i].min;
                int fw = span ? (v - tbl[i].min) * 122 / span : 0;
                oled_bar(2, 2 + i * 2, 124, fw, true);
            }
        }
        break;
    }
    case PAGE_TUNE: {
        draw_header("TUNER");
        tuner_result_t t; tuner_get(&t);
        /* big note name */
        oled_str(8, 2, t.note, false);
        /* frequency line */
        if (t.freq > 0) snprintf(b, sizeof b, "%u.%02u Hz",
                                 (unsigned)t.freq,
                                 (unsigned)(t.freq * 100) % 100);
        else            snprintf(b, sizeof b, "-- Hz");
        oled_str(2, 4, b, false);
        /* cents scale: center tick + moving pointer */
        oled_hline(4, 40, 120, true);
        int px = 64 + (t.cents * 60) / 50;
        if (px < 4) px = 4; if (px > 123) px = 123;
        oled_fill_rect(px, 36, 2, 8, true);
        oled_fill_rect(63, 34, 2, 12, true);      /* center marker */
        snprintf(b, sizeof b, "%+d cents%s", t.cents,
                 t.in_range ? " IN" : "");
        oled_str(2, 6, t.in_range ? "*IN TUNE*" : b, false);
        break;
    }
    }
}

/* --------------------------- events --------------------------- */
static void handle_event(ui_event_t ev)
{
    switch (ev) {
    case EV_ENC_SW_SHORT:
        s_edit = false;
        s_page = (page_t)((s_page + 1) % PAGE_COUNT);
        s_row = 0;
        break;
    case EV_ENC_SW_LONG:
        if (page_rows(s_page)) s_edit = !s_edit;
        break;
    case EV_ENC_CW:
        if (s_edit && page_table(s_page)) {
            const row_t *r = &page_table(s_page)[s_row];
            int v = r->get() + r->step;
            if (v > r->max) v = r->max;
            row_set(r, v);
        } else {
            int nr = page_rows(s_page);
            if (nr) s_row = (s_row + 1) % nr;
        }
        break;
    case EV_ENC_CCW:
        if (s_edit && page_table(s_page)) {
            const row_t *r = &page_table(s_page)[s_row];
            int v = r->get() - r->step;
            if (v < r->min) v = r->min;
            row_set(r, v);
        } else {
            int nr = page_rows(s_page);
            if (nr) s_row = (s_row + nr - 1) % nr;
        }
        break;
    case EV_ACT_SHORT:
        dsp_set_fx(!dsp_get_fx());
        break;
    case EV_ACT_LONG:
        tuner_set_enabled(!tuner_is_enabled());
        s_page = PAGE_TUNE; s_row = 0; s_edit = false;
        break;
    case EV_PRESET_SHORT:
        presets_cycle_next();
        break;
    case EV_PRESET_LONG:
        presets_save_current();
        break;
    default: break;
    }
}

/* --------------------------- task --------------------------- */
static void ui_task(void *arg)
{
    if (!ssd1306_init()) {
        ESP_LOGE(TAG, "OLED init failed — continuing audio-only");
    }
    encoder_init();
    ssd1306_clear();

    /* let the DSP produce its first blocks before the first paint */
    while (!dsp_task_alive()) vTaskDelay(pdMS_TO_TICKS(50));

    uint32_t redraw_ctr = 0;
    const uint32_t redraw_every = CONFIG_UI_PERIOD_MS ? (100 / CONFIG_UI_PERIOD_MS) : 5;

    for (;;) {
        ui_event_t ev;
        while ((ev = encoder_poll()) != EV_NONE)
            handle_event(ev);

        if (++redraw_ctr >= redraw_every) {
            redraw_ctr = 0;
            ssd1306_clear();
            render_pages(dsp_get_fx());
            ssd1306_flush();
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_UI_PERIOD_MS));
    }
}

void ui_start_task(void)
{
    xTaskCreatePinnedToCore(ui_task, "ui", 4096, NULL, PRIO_UI, NULL, CORE_UI);
}
