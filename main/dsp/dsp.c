/*
 * dsp.c — Q15 fixed-point guitar effects.
 *
 *   Reverb : Schroeder topology (4 comb + 4 all-pass + damping one-pole),
 *            realized as a deterministic 32-tap FIR built from the analytic
 *            impulse response at init time (no recursion at run time).
 *   Chorus : sinusoidally modulated delay line, linear interpolation.
 *   Delay  : single-feedback slap-back.
 *   Anti-larsen: RBJ biquad HPF (80..100 Hz) + cubic soft clipper.
 *   Bypass : equal-power crossfade over ~3 ms (no pops/clicks).
 */
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "dsp.h"
#include "tuner.h"
#include "audio_pipe.h"
#include "board.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define TAP_DELAY_S      0.04f     /* reverb pre-delay        */
#define CHORUS_BASE_US   8000      /* chorus nominal delay    */
#define XFADE_BLOCKS     1         /* bypass fade = 1 block (2.66 ms) */
#define CADENCE_TICK_US    (BLOCK_PERIOD_US - 200)  /* wake 200us before deadline */

static dsp_settings_t s_set;
static SemaphoreHandle_t s_param_lock;

static volatile uint16_t s_peak_in, s_peak_out;
volatile uint32_t g_dsp_beat;              /* bumped once per processed block */

/* Block-cadence wake-up for the DSP task (created in dsp_start_task).
 * A PERIODIC timer with ISR dispatch: the expiry callback runs directly from
 * the hardware-timer ISR (affinitised to CPU0 by default), so the audio block
 * clock never depends on the esp_timer daemon task being scheduled. The ISR
 * reschedules itself and releases the DSP task's binary semaphore; both
 * operations are ISR-safe. */
static SemaphoreHandle_t s_wake_sem;
static esp_timer_handle_t s_cadence_timer;

/* One-shot re-arm: esp_timer_stop() is NOT ISR-safe, so instead of a periodic
 * timer we re-start the one-shot from inside its own expiry callback (both
 * calls happen in ISR context, where start-from-callback is explicitly
 * allowed). The 200 us pre-sleep margin below keeps the arm cheap. */
static void IRAM_ATTR cadence_timer_cb(void *arg)
{
    esp_timer_start_once(s_cadence_timer, CADENCE_TICK_US);
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_wake_sem, &hp);
    if (hp) portYIELD_FROM_ISR(hp);
}

/* Live per-block timing stats for the OLED monitor line / host tests.
 * Updated from the core-0 DSP task, read lock-free from core 1 (uint32 torn
 * reads are harmless here — display-only values). */
volatile uint32_t g_dsp_us_max;        /* worst-case stage time this second   */
volatile uint32_t g_dsp_us_last;       /* last processed block's stage time   */
volatile uint32_t g_dsp_us_avg;        /* EWMA (~time constant 64 blocks)     */

/* ------------------------- helpers -------------------------- */
static inline int32_t mul_q15(int32_t x, int32_t y) { return (x * y) >> 15; }

static inline int16_t sat16(int32_t v)
{
    if (v >  32767) return  32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

/* Cubic soft clipper above threshold T: maps [T..FS] -> [T..FS],
 * slope-continuous at T (anti-transient / anti-oscillation guard). */
static inline IRAM_ATTR int16_t soft_clip(int16_t x)
{
    const int32_t t = CONFIG_LIMITER_THRESHOLD;
    const int32_t fs = 32767;
    int32_t a = x < 0 ? -(int32_t)x : (int32_t)x;
    if (a <= t) return x;
    int32_t xn = ((a - t) << 15) / (fs - t);          /* 0..Q15 across range */
    int32_t x2 = mul_q15(xn, xn);
    int32_t x3 = mul_q15(x2, xn);
    int32_t sh = xn - mul_q15(x3, 10923);             /* xn - xn^3/3 shape   */
    int32_t y  = t + mul_q15(fs - t, sh + (sh >> 1)); /* stretch to reach FS */
    if (y > fs) y = fs;
    return sat16(x < 0 ? -y : y);
}

/* ------------------- HPF biquad (RBJ, Q=.707) ------------------- */
typedef struct { int16_t b0, b1, b2, a1, a2; } biquad_t;
static biquad_t s_hpf_c;
static int32_t  s_hpf_z[2];                 /* DF-II transposed states */

static void hpf_compute(float fc)
{
    float w = 6.2831853f * fc / SAMPLE_RATE;
    float cw = cosf(w), sw = sinf(w);
    float alpha = sw / 1.4142f, a0 = 1.0f + alpha;
    s_hpf_c.b0 = sat16(((1.0f + cw) * 0.5f / a0) * 32767.0f);
    s_hpf_c.b1 = sat16((-(1.0f + cw)      / a0) * 32767.0f);
    s_hpf_c.b2 = sat16(((1.0f + cw) * 0.5f / a0) * 32767.0f);
    s_hpf_c.a1 = sat16(((-2.0f * cw)      / a0) * 32767.0f);
    s_hpf_c.a2 = sat16(((1.0f - alpha)    / a0) * 32767.0f);
}

/* Cached flag derived from the per-block snapshot (see dsp_snapshot_params):
 * avoids reading the settings struct inside the sample loop. */
static bool s_hpf_bypass;

/* The settings field stores cutoff in kHz*10 so it fits a uint8_t
 * (80 Hz -> 800, 1000 Hz would overflow). Anything outside the anti-larsen
 * band clamps to 80 Hz; 0 disables the HPF (s_hpf_bypass). */
static inline float hpf_hz_of(uint8_t encoded)
{
    int hz = encoded * 10;
    if (hz < 80)  hz = 80;
    if (hz > 100) hz = 100;
    return (float)hz;
}

static inline IRAM_ATTR int16_t hpf_run(int16_t x)
{
    if (s_hpf_bypass) return x;
    const biquad_t *c = &s_hpf_c;
    int32_t y = mul_q15(c->b0, x) + s_hpf_z[0];
    s_hpf_z[0] = mul_q15(c->b1, x) - mul_q15(c->a1, y) + s_hpf_z[1];
    s_hpf_z[1] = mul_q15(c->b2, x) - mul_q15(c->a2, y);
    return sat16(y);
}

/* --------------------- circular delay lines --------------------- */
static size_t next_pow2(size_t n) { size_t p = 1; while (p < n) p <<= 1; return p; }

static int16_t *alloc_line(size_t samples, size_t *mask_out)
{
    size_t len = next_pow2(samples + BLOCK_SAMPLES + 8);
    int16_t *b = heap_caps_malloc(len * sizeof(int16_t),
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    memset(b, 0, len * sizeof(int16_t));
    *mask_out = len - 1;
    return b;
}

/* ---- slap-back delay ---- */
static int16_t *dly_buf; static size_t dly_mask, dly_pos;
static int16_t  dly_fb_q15; static uint32_t dly_n;   /* whole samples */

static inline IRAM_ATTR int16_t delay_run(int16_t x)
{
    size_t i = dly_pos;
    int16_t echo = dly_buf[(i - dly_n - 1) & dly_mask];
    dly_buf[i] = sat16(x + mul_q15(echo, dly_fb_q15));
    dly_pos = (i + 1) & dly_mask;
    return echo;
}

/* ---- chorus (LFO-modulated delay) ---- */
/* full-cycle sine table, Q15, 64 points: sin(2*pi*i/64)*32767, interpolated */
static const int16_t SINE_Q[64] = {
       0,  3212,  6393,  9512, 12539, 15446, 18204, 20787, 23170, 25329, 27245, 28898,
   30273, 31356, 32137, 32609, 32767, 32609, 32137, 31356, 30273, 28898, 27245, 25329,
   23170, 20787, 18204, 15446, 12539,  9512,  6393,  3212,     0, -3212, -6393, -9512,
  -12539,-15446,-18204,-20787,-23170,-25329,-27245,-28898,-30273,-31356,-32137,-32609,
  -32767,-32609,-32137,-31356,-30273,-28898,-27245,-25329,-23170,-20787,-18204,-15446,
  -12539, -9512, -6393, -3212
};

static int16_t *cho_buf; static size_t cho_mask, cho_pos;
static uint32_t cho_phase, cho_inc;
static int32_t  cho_depth_q16;              /* half-depth in samples, Q16 */

static inline IRAM_ATTR int16_t lfo_value(void)
{
    uint32_t idx  = cho_phase >> 26;                    /* top 6 bits: 0..63 */
    uint32_t frac = (cho_phase >> 20) & 0x3F;           /* next 6 bits       */
    int32_t a = SINE_Q[idx], b = SINE_Q[(idx + 1) & 63];
    return sat16(a + mul_q15(b - a, (int32_t)(frac * 517)));  /* frac*32767/64 */
}

static inline IRAM_ATTR int16_t chorus_run(int16_t x)
{
    cho_buf[cho_pos] = x;
    cho_phase += cho_inc;
    int32_t mod = mul_q15(lfo_value(), cho_depth_q16);            /* Q16 offset */
    uint32_t base = (uint32_t)(CHORUS_BASE_US * SAMPLE_RATE / 1000000) << 16;
    int64_t dv = (int64_t)base + mod;
    if (dv < 0) dv = 0;
    uint32_t d  = (uint32_t)dv;
    uint32_t di = d >> 16, df = d & 0xFFFF;
    size_t i0 = (cho_pos - 1 - di) & cho_mask;
    size_t i1 = (i0 - 1) & cho_mask;
    int32_t s = cho_buf[i0] + mul_q15(cho_buf[i1] - cho_buf[i0], (int32_t)(df >> 1));
    cho_pos = (cho_pos + 1) & cho_mask;
    return sat16(s);
}

/* ------------- Schroeder reverb as 32-tap FIR (analytic IR) ------------- */
static const int COMB_D[4]  = { 1557, 1617, 1491, 1422 };
static const int COMB_G[4]  = { 14855, 14784, 14642, 14561 };   /* ~0.90/0.85 */
static const int ALLP_D[4]  = { 225, 556, 441, 341 };
static const int ALLP_G[4]  = { 11469, 11469, 11469, 11469 };   /* 0.35 */
#define LP_A_Q15 1638                                            /* damping pole */

static int16_t  verb_taps[32];
static uint16_t verb_idx[32];
static int16_t *verb_buf; static size_t verb_mask, verb_pos;
static int16_t  verb_lp;

static void verb_build(void)
{
    int pre = (int)(TAP_DELAY_S * SAMPLE_RATE);
    int ilen = pre + 24000;                       /* covers longest tail */
    int16_t *imp = calloc(ilen, sizeof(int16_t));
    int16_t *tmp = calloc(ilen, sizeof(int16_t));

    /* parallel combs, input scaled by 1/4 to keep headroom */
    for (int i = 0; i < 4; i++) {
        double g = COMB_G[i] / 32767.0;
        int n = (int)(-log(0.001) / log(g));  if (n > 30) n = 30;
        double e = 32767.0;
        for (int k = 0; k <= n; k++) {
            int idx = pre + COMB_D[i] * k;
            imp[idx] = sat16((int32_t)imp[idx] + (int32_t)(e / 4.0));
            e *= g;
        }
    }
    /* fold each cascaded all-pass exactly:
     * H_ap(z) = (-g + z^-D) / (1 - g z^-D)
     * => h_ap[0] = -g ; h_ap[kD] = (1-g^2) g^(k-1), k>=1 */
    for (int a = 0; a < 4; a++) {
        double g = ALLP_G[a] / 32767.0;
        int n = (int)(-log(0.001) / log(g));  if (n > 30) n = 30;
        memset(tmp, 0, ilen * sizeof(int16_t));
        for (int i = 0; i < ilen; i++) {
            if (!imp[i]) continue;
            double x = imp[i];
            tmp[i] = sat16((int32_t)tmp[i] - (int32_t)(g * x));
            double w = x * (1.0 - g * g);
            for (int k = 1; k <= n; k++) {
                int j = i + ALLP_D[a] * k;
                if (j >= ilen) break;
                tmp[j] = sat16((int32_t)tmp[j] + (int32_t)w);
                w *= g;
            }
        }
        memcpy(imp, tmp, ilen * sizeof(int16_t));
    }
    /* damping one-pole y[n]=y[n-1]+A(x[n]-y[n-1]) applied causally to the IR */
    double y = 0;
    for (int i = 0; i < ilen; i++) {
        y += (LP_A_Q15 / 32767.0) * (imp[i] - y);
        imp[i] = sat16((int32_t)y);
    }
    /* pick up to 32 significant taps (threshold scan keeps the energy order) */
    int cnt = 0;
    int thr = 250;
    while (cnt < 32 && thr > 0) {
        for (int i = 0; i < ilen && cnt < 32; i++) {
            int32_t m = abs(imp[i]);
            if (m >= thr) { verb_taps[cnt] = imp[i]; verb_idx[cnt] = (uint16_t)i; cnt++; }
        }
        thr >>= 1;
    }
    while (cnt < 32) { verb_taps[cnt] = 0; verb_idx[cnt] = 0; cnt++; }

    int32_t esum = 0;
    for (int i = 0; i < 32; i++) esum += abs(verb_taps[i]);
    if (esum > 12000) {                            /* conservative wet level */
        for (int i = 0; i < 32; i++) verb_taps[i] = (int16_t)((verb_taps[i] * 12000) / esum);
    }
    free(imp); free(tmp);
}

static inline IRAM_ATTR int16_t verb_run(int16_t x)
{
    verb_buf[verb_pos] = x;
    int32_t acc = 0;
    for (int i = 0; i < 32; i++) {
        acc += (int32_t)verb_taps[i] *
               verb_buf[(verb_pos - 1 - verb_idx[i]) & verb_mask];
    }
    verb_pos = (verb_pos + 1) & verb_mask;
    int16_t wet = sat16(acc >> 15);
    verb_lp = sat16(verb_lp + mul_q15(LP_A_Q15, wet - verb_lp));
    return verb_lp;
}

/* ------------------- failure handling (ARCHITECTURE.md §8) -------------------
 * If a processed block ever takes longer than BLOCK_PERIOD_US to compute, the
 * DSP task cannot keep up with the I2S block clock: we zero the output buffer
 * (safe silence instead of loud garbage), count the overrun for the UI/monitor,
 * and resynchronize by draining one stale input block. */
volatile uint32_t g_dsp_overruns;

static void IRAM_ATTR dsp_fail_silence(int16_t *out)
{
    memset(out, 0, BLOCK_SAMPLES * 2 * sizeof(int16_t));
    g_dsp_overruns++;
}

/* ---------------------------- bypass xfade ---------------------------- */
static int      xf_cnt;                        /* blocks remaining in fade */
static bool     xf_to_fx;
static int16_t  dry_hold[BLOCK_WORDS];         /* last raw input block     */

/* Per-block latched copies (written only by the DSP task inside
 * dsp_snapshot_params(); consumed by dsp_stage_output()). The originals are
 * owned by the UI task, which may start a new fade at any time — the audio
 * path must never observe that mid-block. */
static int      xf_cnt_snap;
static bool     xf_to_fx_snap;

/* Crossfade weights are plain integers scaled by XFADE_BLOCKS so the hot loop
 * never divides: gain for step k of N is (k * 32767 / N), precomputed once. */
static int16_t  xf_gains[XFADE_BLOCKS + 1];

static void xfade_tables_init(void)
{
    for (int k = 0; k <= XFADE_BLOCKS; k++)
        xf_gains[k] = sat16((int32_t)k * 32767 / XFADE_BLOCKS);
}

static inline int16_t xfade_gain(int n)        /* monotone 0..32767 ramp */
{
    if (n < 0) n = 0;
    if (n > XFADE_BLOCKS) n = XFADE_BLOCKS;
    return xf_gains[n];
}

/* ----------------------------- settings ----------------------------- */
void dsp_get_settings(dsp_settings_t *dst)
{
    xSemaphoreTake(s_param_lock, portMAX_DELAY);
    *dst = s_set;
    xSemaphoreGive(s_param_lock);
}

/* Runtime copy of the settings used by the hot path. The UI task writes
 * s_set under s_param_lock; the DSP task snapshots it into s_rt once per
 * block (under the same lock) BEFORE running the stages, so no field can
 * ever tear mid-block and apply_coef() output is always consistent with
 * the gains read during dsp_stage_output(). */
static dsp_settings_t s_rt;

static void IRAM_ATTR dsp_snapshot_params(void)
{
    xSemaphoreTake(s_param_lock, portMAX_DELAY);
    s_rt = s_set;
    /* also latch the bypass-fade state so stage_output never touches vars
     * written by the UI task mid-block */
    xf_cnt_snap   = xf_cnt;
    xf_to_fx_snap = xf_to_fx;
    s_hpf_bypass  = (s_set.hpf_hz == 0);
    xSemaphoreGive(s_param_lock);
}

static void apply_coef(void)
{
    dly_fb_q15 = (int16_t)((int32_t)s_set.delay_fb * 32767 / 100 * 9 / 10); /* cap 90% */
    dly_n = (uint32_t)s_set.delay_ms * SAMPLE_RATE / 1000;
    if (dly_n > dly_mask) dly_n = dly_mask;
    cho_inc = (uint32_t)(((double)s_set.chorus_rate_mhz / 1000.0 / SAMPLE_RATE)
                         * 4294967295.99);
    cho_depth_q16 = (int32_t)((double)s_set.chorus_depth_us / 1e6
                              * SAMPLE_RATE * 65536.0 / 2.0);
    hpf_compute(hpf_hz_of(s_set.hpf_hz));
}

void dsp_set_field(int field_id, int value)
{
    xSemaphoreTake(s_param_lock, portMAX_DELAY);
    switch (field_id) {
    case FLD_DELAY_MIX:    s_set.delay_mix       = value; break;
    case FLD_DELAY_MS:     s_set.delay_ms        = value; break;
    case FLD_DELAY_FB:     s_set.delay_fb        = value; break;
    case FLD_CHORUS_MIX:   s_set.chorus_mix      = value; break;
    case FLD_CHORUS_RATE:  s_set.chorus_rate_mhz = value; break;
    case FLD_CHORUS_DEPTH: s_set.chorus_depth_us = value; break;
    case FLD_REVERB_MIX:   s_set.reverb_mix      = value; break;
    case FLD_HPF:          s_set.hpf_hz          = value; break;
    case FLD_MASTER:       s_set.master_pct      = value; break;
    default: break;
    }
    apply_coef();
    xSemaphoreGive(s_param_lock);
}

void dsp_set_fx(bool on)
{
    xSemaphoreTake(s_param_lock, portMAX_DELAY);
    if (s_set.fx_on != on) {             /* don't restart a fade that isn't needed */
        s_set.fx_on = on;
        xf_cnt = XFADE_BLOCKS;
        xf_to_fx = on;
    }
    xSemaphoreGive(s_param_lock);
}

void dsp_apply_settings(const dsp_settings_t *src)
{
    xSemaphoreTake(s_param_lock, portMAX_DELAY);
    bool fx_changed = (src->fx_on != s_set.fx_on);
    s_set = *src;
    apply_coef();
    if (fx_changed) { xf_cnt = XFADE_BLOCKS; xf_to_fx = src->fx_on; }
    xSemaphoreGive(s_param_lock);
}

bool dsp_get_fx(void)
{
    xSemaphoreTake(s_param_lock, portMAX_DELAY);
    bool v = s_set.fx_on;
    xSemaphoreGive(s_param_lock);
    return v;
}

void dsp_set_preset(int idx)
{
    xSemaphoreTake(s_param_lock, portMAX_DELAY);
    s_set.preset = (int8_t)idx;
    xSemaphoreGive(s_param_lock);
}

/* ------------------------------- init ------------------------------- */
void dsp_init(void)
{
    /* The DSP task snapshots the settings every block, so the lock must
     * already exist when dsp_init() returns. Created here (not in
     * dsp_start_task) to keep an init-order hazard out of main.c. */
    s_param_lock = xSemaphoreCreateMutex();

    s_set.preset = 0; s_set.fx_on = true;
    s_set.delay_mix       = CONFIG_DELAY_MIX_PCT_DFLT;
    s_set.chorus_mix      = CONFIG_CHORUS_MIX_PCT_DFLT;
    s_set.reverb_mix      = CONFIG_REVERB_MIX_PCT_DFLT;
    s_set.delay_ms        = CONFIG_DELAY_TIME_MS_DFLT;
    s_set.delay_fb        = CONFIG_DELAY_FEEDBACK_PCT_DFLT;
    s_set.chorus_rate_mhz = CONFIG_CHORUS_RATE_MHZ;
    s_set.chorus_depth_us = CONFIG_CHORUS_DEPTH_US;
    s_set.hpf_hz          = CONFIG_HPF_CUTOFF_HZ * 10;   /* kHz*10 encoding */
    s_set.master_pct      = CONFIG_OUTPUT_GAIN_PCT;

    dly_buf  = alloc_line((size_t)(0.4f * SAMPLE_RATE), &dly_mask);
    cho_buf  = alloc_line((size_t)(0.02f * SAMPLE_RATE), &cho_mask);
    verb_buf = alloc_line((size_t)(TAP_DELAY_S * SAMPLE_RATE) + 24000, &verb_mask);
    assert(dly_buf && cho_buf && verb_buf);   /* internal-RAM lines are fatal */

    xfade_tables_init();
    verb_build();
    apply_coef();
    s_rt = s_set;                        /* prime the runtime snapshot */
    tuner_init();
}

/* Block scratch buffers: file-scope so they can live in internal RAM via
 * heap_caps_malloc (plain `static` arrays would land in .bss, which on this
 * build sits in PSRAM-mapped address space — unacceptable inside the core-0
 * audio path). Sizes are fixed at compile time, allocated once in
 * dsp_start_task(), never reallocated while the task runs. */
static int32_t *raw_i, *raw_o;        /* interleaved 32-bit DMA slots   */
static int16_t *q15_i, *q15_o;        /* interleaved stereo Q15 frames  */
static int16_t *q15_m;                /* mono post-front-end Q15        */
static int16_t s_wd[BLOCK_SAMPLES], s_wc[BLOCK_SAMPLES], s_wv[BLOCK_SAMPLES];
                                      /* wet-send scratch (stage output)*/

/* Forward declaration: the core-0 block-cadence task is defined below. */
static void dsp_task(void *arg);

void dsp_start_task(void)
{
    /* Block scratch in internal RAM (see note above the pointer decls). */
    raw_i  = heap_caps_malloc(BLOCK_WORDS * sizeof(int32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    raw_o  = heap_caps_malloc(BLOCK_WORDS * sizeof(int32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    q15_i  = heap_caps_malloc(BLOCK_WORDS * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    q15_o  = heap_caps_malloc(BLOCK_WORDS * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    q15_m  = heap_caps_malloc(BLOCK_SAMPLES * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(raw_i && raw_o && q15_i && q15_o && q15_m);

    s_wake_sem = xSemaphoreCreateBinary();
    const esp_timer_create_args_t ta = { .callback = cadence_timer_cb,
                                         .dispatch_method = ESP_TIMER_ISR,
                                         .name = "dsp_cadence" };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_cadence_timer));

    /* The task body lives in IRAM (IRAM_ATTR below); its stack is allocated
     * from internal RAM explicitly so no part of the audio domain ever
     * depends on PSRAM (ARCHITECTURE.md §7). */
    StaticTask_t *tcb = heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL);
    StackType_t *stk  = heap_caps_malloc(4096, MALLOC_CAP_INTERNAL);
    assert(tcb && stk);
    xTaskCreateStaticPinnedToCore(dsp_task, "dsp", 4096, NULL, PRIO_DSP,
                                  stk, tcb, CORE_AUDIO);
}

bool dsp_task_alive(void)
{
    return g_dsp_beat != 0;                /* false until the first block ran */
}

uint16_t dsp_peak_in(void)  { return s_peak_in;  }
uint16_t dsp_peak_out(void) { return s_peak_out; }

void dsp_reset_timing_stats(void)
{
    g_dsp_us_max = 0;
    g_dsp_overruns = 0;
}

/* PCM1808/MAX98357A are configured as I2S masters with 32-bit Philips slots:
 * the ADC right-justifies its 24 valid bits in the slot (bits 3..26), so we
 * take bits 14..29 for a Q15 value that sits 2 dB below full scale — exactly
 * the acoustic headroom the anti-feedback budget asks for. The TX conversion
 * mirrors this by re-right-justifying into bits 3..26. */
static inline IRAM_ATTR int16_t q15_from_i32(int32_t v)
{
    return sat16(v >> SAMPLE_SHIFT);
}

static void IRAM_ATTR convert_int32_back(const int16_t *src, int32_t *dst, size_t words)
{
    for (size_t i = 0; i < words; i++) dst[i] = (int32_t)src[i] << SAMPLE_SHIFT;
}

static void IRAM_ATTR convert_q15(const int32_t *src, int16_t *dst, size_t frames)
{
    for (size_t i = 0; i < 2 * frames; i++) dst[i] = q15_from_i32(src[i]);
}

/* ------------------------- DSP task (core 0) -------------------------
 * Owns the audio block clock: one iteration per BLOCK_PERIOD_US. Reads a raw
 * int32 stereo block from the RX ring, converts to Q15, snapshots the UI
 * settings once per block, runs the two pipeline stages under a hard deadline
 * watch, and hands the result to the TX ring. No allocations, no logging, no
 * flash access in this loop. */
static void dsp_task(void *arg)
{

    /* Fade-in over the first few blocks so the exciter never gets a step. */
    int warm = 4;
    uint32_t deadline_us = esp_timer_get_time() + BLOCK_PERIOD_US;
    bool armed = false;                       /* cadence timer currently pending */

    for (;;) {
        if (gtr_ring_read_timed(&g_audio.in_ring, raw_i, BYTES_PER_BLOCK,
                                pdMS_TO_TICKS(20)) != BYTES_PER_BLOCK) {
            /* I2S RX stalled (no ADC data): emit silence instead of garbage. */
            memset(raw_o, 0, BYTES_PER_BLOCK);
            gtr_ring_write(&g_audio.out_ring, raw_o, BYTES_PER_BLOCK);
            continue;
        }
        convert_q15(raw_i, q15_i, BLOCK_SAMPLES);

        /* Snapshot UI-written settings once per block: no field can tear
         * mid-block and coefficients stay consistent with the gains below. */
        dsp_snapshot_params();

        int64_t t0 = esp_timer_get_time();
        bool tuner_on = tuner_is_enabled();
        dsp_stage_input(q15_i, q15_m, BLOCK_SAMPLES, tuner_on);
        dsp_stage_output(q15_m, q15_i, q15_o, BLOCK_SAMPLES);
        int64_t elapsed = esp_timer_get_time() - t0;

        if (elapsed > BLOCK_PERIOD_US) {           /* §8 failure handling */
            dsp_fail_silence(q15_o);               /* zero output, count it */
        }
        g_dsp_us_last = (uint32_t)elapsed;
        if ((uint32_t)elapsed > g_dsp_us_max) g_dsp_us_max = (uint32_t)elapsed;
        g_dsp_us_avg += ((uint32_t)elapsed - g_dsp_us_avg) / 64;   /* EWMA */

        if (warm > 0) {                            /* startup fade-in ramp */
            int g = (4 - warm + 1) * 8192;         /* ~0.25..1.0 Q15 */
            for (int i = 0; i < BLOCK_WORDS; i++)
                q15_o[i] = sat16(mul_q15(q15_o[i], g));
            warm--;
        }
        convert_int32_back(q15_o, raw_o, BLOCK_WORDS);
        gtr_ring_write(&g_audio.out_ring, raw_o, BYTES_PER_BLOCK);
        g_dsp_beat++;

        /* Hold the block cadence (absolute-time: zero long-term drift).
         * Arm the ISR-dispatched one-shot 200 us before the deadline; the
         * expiry callback releases our semaphore AND self-re-arms, so after
         * the first arm the loop just waits. If the read timeout consumed
         * the whole period (RX stalled), the timer was never armed or has
         * no pending give — fall back to a plain timed wait to stay alive. */
        deadline_us += BLOCK_PERIOD_US;
        int64_t now = esp_timer_get_time();
        if (deadline_us - now > 500) {
            if (!armed) {
                esp_timer_start_once(s_cadence_timer, deadline_us - now - 200);
                armed = true;
            }
            xSemaphoreTake(s_wake_sem, portMAX_DELAY);
            armed = false;                    /* expiry ran (or raced); re-arm next pass */
        } else if (now - deadline_us > 4 * BLOCK_PERIOD_US) {
            deadline_us = now;                /* resync after a big stall */
            esp_timer_stop(s_cadence_timer);  /* task ctx: stop is safe here */
            armed = false;
        } else {
            /* behind schedule but within tolerance: process back-to-back */
            armed = false;
            esp_timer_stop(s_cadence_timer);
        }
    }
}

/* ====================== public block processors ======================
 * The pipeline is split into two IRAM-resident stages so that main.c can
 * time each independently against the 2.66 ms block budget:
 *   stage 1: HPF -> tuner tap -> limiter        (anti-feedback front end)
 *   stage 2: effects rack -> mixer -> gain/xfade -> output buffer
 * `in`/`out` are interleaved stereo Q15 frames of BLOCK_SAMPLES pairs. */
void IRAM_ATTR dsp_stage_input(const int16_t *in, int16_t *mono, int n,
                               bool tuner_on)
{
    uint32_t peak = 0;
    for (int i = 0; i < n; i++) {
        int32_t m = ((int32_t)in[2*i] + in[2*i+1]) >> 1;
        uint32_t a = m < 0 ? -(uint32_t)m : (uint32_t)m;
        if (a > peak) peak = a;
        /* ARCHITECTURE.md order: HPF first (cut sub-bass feedback energy),
         * then the fast limiter catches transients before the FX rack. */
        mono[i] = soft_clip(hpf_run((int16_t)m));
    }
    s_peak_in = (uint16_t)peak;
#if CONFIG_TUNER_ENABLED
    /* Tuner tap on the PRE-EFFECTS signal (requirement 5.4). The analysis
     * itself is throttled internally (TUNER_INTERVAL blocks), so this call
     * costs only a few hundred cycles on most blocks. */
    if (tuner_on) tuner_push_frame(mono, n);
#else
    (void)tuner_on;
#endif
}

void IRAM_ATTR dsp_stage_output(const int16_t *mono, const int16_t *in,
                                int16_t *out, int n)
{
    /* wet sends land in file-scope internal-RAM buffers (no stack VLA-style
     * frame pressure inside the IRAM task; see dsp_start_task allocation) */
    int16_t *wd = s_wd, *wc = s_wc, *wv = s_wv;

    /* ---- effects rack: Chorus -> Delay -> Reverb (parallel sends) ---- */
    for (int i = 0; i < n; i++) {
        wc[i] = chorus_run(mono[i]);
        wd[i] = delay_run(mono[i]);
        wv[i] = verb_run(mono[i]);
    }

    /* ---- wet/dry mixer + master gain (all from the per-block snapshot) ---- */
    const int32_t third = 10923;          /* ~32767/3: unity when all at 100% */
    int32_t gd = mul_q15(s_rt.delay_mix  * 327, third);   /* Q15 wet weights */
    int32_t gc = mul_q15(s_rt.chorus_mix * 327, third);
    int32_t gv = mul_q15(s_rt.reverb_mix * 327, third);
    int32_t mg = s_rt.master_pct * 327;                   /* Q15 master gain */
    bool fxon = s_rt.fx_on;

    uint32_t peak_o = 0;
    for (int i = 0; i < n; i++) {
        int32_t wet = mul_q15(wd[i], gd) + mul_q15(wc[i], gc) + mul_q15(wv[i], gv);
        wet = sat16(wet);
        /* stereo spread: right channel gets 80% of the wet bus (width cue) */
        int32_t ol  = mul_q15(mono[i] + wet, mg);
        int32_t or_ = mul_q15(mono[i] + sat16(wet - mul_q15(wet, 6554)), mg);

        if (xf_cnt_snap > 0) {             /* click-free bypass transition */
            int16_t g = xfade_gain(XFADE_BLOCKS - xf_cnt_snap + 1);
            int16_t k = sat16(32767 - g);
            int16_t fg = xf_to_fx_snap ? g : k;      /* weight of processed path */
            int16_t dg = xf_to_fx_snap ? k : g;      /* weight of dry path       */
            ol  = mul_q15(ol,  fg) + mul_q15(dry_hold[2*i],   dg);
            or_ = mul_q15(or_, fg) + mul_q15(dry_hold[2*i+1], dg);
        } else if (!fxon) {                /* true bypass: raw ADC frames */
            ol = in[2*i]; or_ = in[2*i+1];
        }
        out[2*i]   = sat16(ol);
        out[2*i+1] = sat16(or_);
        uint32_t ao = (uint32_t)(ol < 0 ? -ol : ol);
        if (ao > peak_o) peak_o = ao;
    }
    if (xf_cnt_snap > 0) xf_cnt_snap--;
    memcpy(dry_hold, in, n * 2 * sizeof(int16_t));
    s_peak_out = (uint16_t)peak_o;
}

/* One-shot convenience wrapper (host tests / bring-up): full chain. */
void IRAM_ATTR dsp_process_block(const int16_t *in, int16_t *out, bool tuner_on)
{
    int16_t mono[BLOCK_SAMPLES];
    dsp_stage_input(in, mono, BLOCK_SAMPLES, tuner_on);
    dsp_stage_output(mono, in, out, BLOCK_SAMPLES);
}
