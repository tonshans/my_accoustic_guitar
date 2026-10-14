/*
 * battery.h — 18650 voltage monitor on GPIO2 (ADC1_CH1) through a resistor
 * divider. Polled slowly from the core-1 health task; never touches core 0.
 */
#ifndef __GTR_BATTERY_H__
#define __GTR_BATTERY_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void battery_init(void);

/* Refresh one raw sample set (median of 9) and update the cached state.
 * Call every ~1 s from the monitor task. */
void battery_sample(void);

uint16_t battery_mv(void);     /* battery voltage in mV (divider-corrected) */
uint8_t  battery_pct(void);    /* 0..100 between BAT_EMPTY_MV..BAT_FULL_MV  */
bool     battery_low(void);    /* true below ~20% — warn on OLED            */

#ifdef __cplusplus
}
#endif

#endif
