/*
 * tuner.c — autocorrelation chromatic tuner.
 *
 * CPU budget: analysis window 1160 samples @22.05 kHz, lags 44..1100 with a
 * coarse->fine search => ~3k MACs per attempt, every TUNER_INTERVAL blocks.
 * All integer; the only float work happens once per attempt (note mapping).
 */
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "tuner.h"
#include "board.h"

#define TUNER_FS          (SAMPLE_RATE / 2)        /* 24 000 Hz after decimator */
#define WIN               1160                     /* ~52 ms buffer             */
#define LAG_MIN           46                       /* 522 Hz max (above high E) */
#define LAG_MAX           1150                     /* 20.9 Hz min (< low E 41 Hz)*/
#define GATE_ON           800                      /* Q15 open threshold        */
#define GATE_OFF          400                      /* hysteresis close          */
#define TUNER_INTERVAL    4                        /* analyze every N blocks (~10.6 ms) */
#define IN_TUNE_CENTS     2
#define A4_HZ             440.0f

static int16_t  buf[WIN];
static uint32_t fill;              /* valid samples in buf (< WIN until primed) */
static uint32_t wr;                /* ring write index                          */
static int32_t  lp_state;
static uint16_t frame_peak;
static uint32_t block_ctr;
static bool     gate_open;
static volatile bool s_enabled = true;   /* UI toggles; DSP task reads      */

static tuner_result_t res;
static float hist[5]; static int hist_n, hist_i;

void tuner_init(void)
{
    memset(buf, 0, sizeof(buf));
    fill = wr = 0; lp_state = 0; frame_peak = 0; block_ctr = 0;
    gate_open = false;
    memset(&res, 0, sizeof(res));
    strcpy(res.note, "--");
    hist_n = hist_i = 0;
}

static inline int16_t sat16_in(int32_t v)
{
    if (v >  32767) return  32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

static inline void decimate_push(const int16_t *mono, int n)
{
    /* 2x decimation with one-pole anti-alias LP, alpha = 1/16 */
    for (int i = 0; i + 1 < n; i += 2) {
        int32_t x = ((int32_t)mono[i] + mono[i+1]) >> 1;
        lp_state += (x - (lp_state >> 4));          /* pole at fc ~= fs/10     */
        buf[wr] = sat16_in(lp_state >> 4);          /* true LP output          */
        wr = (wr + 1) % WIN;
        if (fill < WIN) fill++;
    }
}

/* linear access to the last `n` samples in chronological order */
static inline int16_t win_at(int idx)               /* idx 0..WIN-1 oldest->newest */
{
    return buf[(wr - (int)fill + idx + WIN * 2) % WIN];
}

static uint32_t xcorr_sum(int lag, int n)
{
    /* correlation over the newest n samples; negative values clamp to 0 so
     * the u32 comparison logic stays valid */
    const int base = (int)fill - n;
    int64_t acc = 0;
    for (int i = 0; i < n - lag; i++) {
        acc += (int32_t)win_at(base + i) * win_at(base + i + lag);
    }
    if (acc < 0) acc = 0;
    return (uint32_t)(acc >> 14);                   /* keep in u32 range       */
}

static void note_map(float f)
{
    static const char *names[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
    if (f <= 0) { strcpy(res.note, "--"); res.freq = 0; res.cents = 0; return; }
    float n = 69.0f + 12.0f * log2f(f / A4_HZ);
    int ni = (int)lroundf(n);
    float cents_f = (n - (float)ni) * 100.0f;
    if (cents_f >  50) cents_f =  50;
    if (cents_f < -50) cents_f = -50;
    res.cents = (int8_t)cents_f;
    int pc = ((ni % 12) + 12) % 12;
    int octave = ni / 12 - 1;
    snprintf(res.note, sizeof(res.note), "%s%d", names[pc], octave);
    res.in_range = (abs(res.cents) <= IN_TUNE_CENTS);
}

static void push_hist(float f)
{
    hist[hist_i] = f; hist_i = (hist_i + 1) % 5;
    if (hist_n < 5) hist_n++;
    float tmp[5]; memcpy(tmp, hist, sizeof(hist));
    for (int i = 0; i < hist_n; i++)                 /* insertion sort median */
        for (int j = i + 1; j < hist_n; j++)
            if (tmp[j] < tmp[i]) { float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
    res.freq = tmp[hist_n / 2];
}

static void analyze(void)
{
    /* energy check on newest half window */
    uint32_t e = 0;
    int n = WIN;
    for (int i = 0; i < n; i += 8) {
        int32_t v = win_at(i); e += (uint32_t)(v * v);
    }
    if (e == 0) return;

    /* coarse scan (step 4) then refine +-6 around best */
    uint32_t best = 0; int best_lag = 0;
    for (int lag = LAG_MIN; lag <= LAG_MAX; lag += 4) {
        uint32_t c = xcorr_sum(lag, n);
        if (c > best) { best = c; best_lag = lag; }
    }
    if (best == 0) return;                          /* no periodic content     */
    int lo = best_lag - 6, hi = best_lag + 6;
    if (lo < LAG_MIN) lo = LAG_MIN;
    if (hi > LAG_MAX) hi = LAG_MAX;
    for (int lag = lo; lag <= hi; lag++) {
        uint32_t c = xcorr_sum(lag, n);
        if (c > best) { best = c; best_lag = lag; }
    }

    /* parabolic interpolation around best_lag */
    uint32_t cm = xcorr_sum(best_lag - 1, n);
    uint32_t cp = xcorr_sum(best_lag + 1, n);
    float denom = (float)((int32_t)cm - 2 * (int32_t)best + (int32_t)cp);
    float d = 0.0f;
    if (denom != 0.0f) d = 0.5f * ((float)cm - (float)cp) / denom;
    if (d >  0.5f) d =  0.5f;
    if (d < -0.5f) d = -0.5f;
    float lag_f = (float)best_lag + d;

    /* octave disambiguation: guitar fundamentals are weak, harmonics strong.
     * If a LONGER period (2x) correlates nearly as well as the best one, the
     * true fundamental is an octave lower -> prefer it. If instead the half
     * period wins outright, correct upward. */
    if (best_lag * 2 <= LAG_MAX) {
        uint32_t c_dbl = xcorr_sum(best_lag * 2, n);
        if ((uint64_t)c_dbl * 100 >= (uint64_t)best * 78) {
            lag_f *= 2.0f;                          /* true period is longer   */
        } else if (best_lag / 2 >= LAG_MIN) {
            uint32_t c_half = xcorr_sum(best_lag / 2, n);
            if (c_half > best) lag_f *= 0.5f;       /* detected sub-harmonic   */
        }
    }

    float f = (float)TUNER_FS / lag_f;
    push_hist(f);
    note_map(res.freq ? res.freq : f);
}

void tuner_push_frame(const int16_t *mono, int n)
{
    uint32_t pk = 0;
    for (int i = 0; i < n; i++) {
        uint32_t a = mono[i] < 0 ? -(uint32_t)mono[i] : (uint32_t)mono[i];
        if (a > pk) pk = a;
    }
    /* peak decay so the UI bar reacts smoothly */
    if (pk < frame_peak) pk = frame_peak - (frame_peak - pk + 1) / 2;
    frame_peak = (uint16_t)(pk > 32767u ? 32767u : pk);   /* Q15: clamp to int16 span */
    res.peak = frame_peak;

    if (!gate_open && pk > GATE_ON)  gate_open = true;
    if ( gate_open && pk < GATE_OFF) gate_open = false;

    decimate_push(mono, n);

    if (++block_ctr < TUNER_INTERVAL) return;
    block_ctr = 0;

    if (!gate_open || fill < WIN) {
        if (!gate_open && res.freq) { /* decay display after silence */
            hist_n = hist_i = 0; res.freq = 0; strcpy(res.note, "--");
            res.cents = 0; res.in_range = false;
        }
        return;
    }
    analyze();
}

void tuner_get(tuner_result_t *dst)
{
    *dst = res;
}

void tuner_set_enabled(bool on) { s_enabled = on; }
bool tuner_is_enabled(void)     { return s_enabled; }
