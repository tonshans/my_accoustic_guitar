/*
 * tuner.h — offline chromatic tuner fed with PRE-EFFECTS audio (req. 5.4).
 *
 * Algorithm: hysteresis gate -> 2x decimator + one-pole LP -> autocorrelation
 * (integer) over a 1160-sample window at 24 000 Hz -> parabolic peak ->
 * octave disambiguation by cross-correlation with half/double period ->
 * median smoothing -> note name + cents.
 * Runs inside the DSP task on core 0 (deterministic: bounded integer MACs,
 * no allocations, no blocking); an analysis is attempted every
 * TUNER_INTERVAL blocks (~10.6 ms).
 */
#ifndef __GTR_TUNER_H__
#define __GTR_TUNER_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t peak;        /* Q15 input peak of last frame            */
    float    freq;        /* detected fundamental (Hz), 0 = not ready*/
    int8_t   cents;       /* -50..+50 deviation from nearest note    */
    char     note[8];     /* e.g. "A2"; "--" while gated off         */
    bool     in_range;    /* |cents| <= IN_TUNE_CENTS                */
} tuner_result_t;

void tuner_init(void);

/* Push one mono block (n Q15 samples, typically BLOCK_SAMPLES). Internally
 * decimates/accumulates and runs the periodic analysis every few calls.
 * Cheap enough to be called from the audio task on every block. */
void tuner_push_frame(const int16_t *mono, int n);

/* Latest result (safe copy for the UI task). */
void tuner_get(tuner_result_t *dst);

/* Tuner on/off flag — written by the UI (BTN_ACT long press), read once per
 * audio block by the core-0 DSP task. Single bool, no tearing risk. */
void tuner_set_enabled(bool on);
bool tuner_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif
