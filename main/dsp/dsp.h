/*
 * dsp.h — Q15 fixed-point guitar effects chain (runs on core 0, IRAM).
 *
 *   in -> HPF(80-100 Hz) -> soft limiter -> { dry | slap delay | chorus | reverb }
 *      -> wet/dry mix -> master gain -> out
 *
 * The tuner taps the PRE-EFFECTS signal (requirement 5.4) inside stage A.
 *
 * Determinism rules inside dsp_process_block():
 *   - no malloc/free, no logging, no locks, no float ops in the sample loop
 *   - all state lives in internal RAM; hot code is IRAM-resident
 */
#ifndef __GTR_DSP_H__
#define __GTR_DSP_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* UI-visible / persisted settings. The UI task writes them under
 * dsp_param_mutex(); the audio task snapshots them once per block. */
typedef struct {
    int8_t   preset;                    /* index into preset table            */
    bool     fx_on;                     /* false = bypass (crossfaded)        */
    uint8_t  delay_mix, chorus_mix, reverb_mix;   /* wet mix 0..100 %         */
    uint16_t delay_ms;                  /* slap-back time, 20..400 ms         */
    uint8_t  delay_fb;                  /* feedback 0..90 %                   */
    uint16_t chorus_rate_mhz;           /* LFO rate in millihertz 100..2000   */
    uint16_t chorus_depth_us;           /* modulation depth in microseconds   */
    uint8_t  hpf_hz;                    /* cutoff kHz*10: 800/900/1000 (0=off) */
    uint8_t  master_pct;                /* output gain %, conservative        */
} dsp_settings_t;

void dsp_init(void);

/* Create + pin the core-0 DSP task (after audio_pipe_init and first preset
 * load, before audio_pipe_enable_tx). */
void dsp_start_task(void);

/* Split pipeline (each call handles n stereo Q15 frames, n <= BLOCK_SAMPLES).
 * Called only from the core-0 DSP task; timed separately against the block
 * budget by main.c (ARCHITECTURE.md §8 failure handling).
 *   stage 1: HPF -> tuner tap -> limiter
 *   stage 2: effects rack -> wet/dry mixer -> master gain -> bypass xfade */
void dsp_stage_input(const int16_t *in, int16_t *mono, int n, bool tuner_on);
void dsp_stage_output(const int16_t *mono, const int16_t *in, int16_t *out, int n);

/* Convenience full-chain wrapper (host tests / bring-up). */
void dsp_process_block(const int16_t *in, int16_t *out, bool tuner_on);

/* Incremented whenever a processed block exceeded the block deadline and the
 * output was forced to silence (should stay at 0 during normal playing). */
extern volatile uint32_t g_dsp_overruns;

/* Per-block stage-time statistics measured on core 0 (microseconds):
 * last / max-since-reset / EWMA. Display-only, read lock-free from core 1. */
extern volatile uint32_t g_dsp_us_last;
extern volatile uint32_t g_dsp_us_max;
extern volatile uint32_t g_dsp_us_avg;
void dsp_reset_timing_stats(void);

/* Bumped once per processed block — health watchdog for core 1. */
extern volatile uint32_t g_dsp_beat;

/* True once the DSP task has completed at least one block. */
bool dsp_task_alive(void);

/* Snapshot of live settings (copy — safe from any task). */
void dsp_get_settings(dsp_settings_t *dst);

/* Replace all settings at once (preset load path; runs on core 1, takes the
 * param mutex, recomputes coefficients — never mid-sample). */
void dsp_apply_settings(const dsp_settings_t *src);

/* Field ids used by the UI page descriptors (dsp_set_field): */
enum {
    FLD_DELAY_MIX, FLD_DELAY_MS, FLD_DELAY_FB,
    FLD_CHORUS_MIX, FLD_CHORUS_RATE, FLD_CHORUS_DEPTH,
    FLD_REVERB_MIX, FLD_HPF, FLD_MASTER,
};

/* Update one field + recompute filter coefficients. Safe from UI task
 * (takes the param mutex internally; coefficient update happens at the
 * next block boundary via a dirty flag — never mid-sample). */
void dsp_set_field(int field_id, int value);

void dsp_set_fx(bool on);               /* triggers a click-free crossfade */
bool dsp_get_fx(void);                  /* current bypass state            */

/* Preset index bookkeeping (contents live in presets.c; this only mirrors
 * the index so the DSP/UI agree without extra locking). */
void dsp_set_preset(int idx);

/* Metering for the OLED level bar (peak of last processed block, Q15). */
uint16_t dsp_peak_in(void);
uint16_t dsp_peak_out(void);

#ifdef __cplusplus
}
#endif

#endif
