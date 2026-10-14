/*
 * tuner.h — offline chromatic tuner fed with PRE-EFFECTS audio (req. 5.4).
 *
 * Algorithm: hysteresis gate -> 2x decimator + one-pole LP -> autocorrelation
 * (integer) over a 1160-sample window at 22 050 Hz -> parabolic peak ->
 * octave disambiguation by cross-correlation with half/double period ->
 * median smoothing -> note name + cents.
 * Runs inside the DSP task every 2 blocks (~5.8 ms of audio per analysis,
 * an analysis is attempted every TUNER_INTERVAL blocks).
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

/* Push one mono block (BLOCK_SAMPLES Q15 samples). Internally accumulates
 * frames and runs the periodic analysis every few calls. Cheap enough to be
 * called from the audio task on every block. */
void tuner_push_frame(const int16_t *mono);

/* Latest result (safe copy for the UI task). */
void tuner_get(tuner_result_t *dst);

#ifdef __cplusplus
}
#endif

#endif
