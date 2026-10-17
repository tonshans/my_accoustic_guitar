/*
 * presets.h — 4 user presets persisted in NVS ("gtr" namespace).
 * All flash/NVS access happens on core 1 only (requirement 3.2: the DSP task
 * never touches files); the current settings are pushed into the DSP with
 * dsp_apply_settings(), which copies under the parameter mutex.
 */
#ifndef __GTR_PRESETS_H__
#define __GTR_PRESETS_H__

#include "dsp.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_PRESETS 4

/* Load preset `idx` from NVS into `out` (falls back to Kconfig defaults if
 * the slot was never saved). Returns the index actually loaded. */
int  presets_load(int idx, dsp_settings_t *out);

/* Persist `s` into slot `idx`. */
void presets_save(int idx, const dsp_settings_t *s);

/* UI helpers: cycle forward through slots and apply; save current into slot. */
void presets_cycle_next(void);
void presets_cycle_prev(void);
void presets_save_current(void);
int  presets_current(void);

#ifdef __cplusplus
}
#endif

#endif
