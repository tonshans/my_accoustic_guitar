/*
 * encoder.h — rotary encoder (A=GPIO10, B=GPIO11, SW=GPIO12) plus the two
 * push buttons (BTN_ACT=GPIO13, BTN_PRESET=GPIO14). Polled by the UI task at
 * CONFIG_UI_PERIOD_MS with a 2-tap debounce; no interrupts, no shared ISRs.
 */
#ifndef __GTR_ENCODER_H__
#define __GTR_ENCODER_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EV_NONE = 0,
    EV_ENC_CW,          /* rotary clockwise detent                */
    EV_ENC_CCW,         /* rotary counter-clockwise detent        */
    EV_ENC_SW_SHORT,    /* encoder push, released < LONG_PRESS_MS */
    EV_ENC_SW_LONG,     /* encoder push held >= LONG_PRESS_MS     */
    EV_ACT_SHORT,       /* button 1: bypass toggle                */
    EV_ACT_LONG,        /* button 1 held: tuner on/off            */
    EV_PRESET_SHORT,    /* button 2: cycle preset                 */
    EV_PRESET_LONG,     /* button 2 held: save preset to NVS      */
} ui_event_t;

void       encoder_init(void);
ui_event_t encoder_poll(void);        /* call every UI tick; one event max */

#ifdef __cplusplus
}
#endif

#endif
