/*
 * audio_pipe.h — I2S0 RX (PCM1808) / I2S1 TX (MAX98357A) plumbing.
 *
 * DMA callbacks are IRAM-resident and only copy one block between the I2S
 * driver and two lock-free rings; all processing happens in the DSP task.
 */
#ifndef __AUDIO_PIPE_H__
#define __AUDIO_PIPE_H__

#include "ringbuffer.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gtr_ring_t in_ring;     /* producer: RX DMA cb   -> consumer: DSP task */
    gtr_ring_t out_ring;    /* producer: DSP task    -> consumer: TX DMA cb */
    volatile uint32_t rx_overruns;   /* ADC data dropped (DSP too slow) */
    volatile uint32_t tx_underruns;  /* DAC starved (silence inserted)  */
} audio_pipe_t;

extern audio_pipe_t g_audio;

esp_err_t audio_pipe_init(void);

/* Pre-fill the output ring with `blocks` silent blocks so the TX DMA never
 * sees an empty buffer at start-up (pop/click prevention). Call BEFORE
 * audio_pipe_enable_tx(). */
void audio_pipe_prefill_silence(int blocks);

/* Start/stop the TX side (MAX98357A). Enabled only after the silence prefill;
 * disabled again during shutdown fade-out to avoid a power-down thump. */
void audio_pipe_enable_tx(void);
void audio_pipe_disable_tx(void);

#ifdef __cplusplus
}
#endif

#endif
