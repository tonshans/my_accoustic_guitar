/*
 * ui.h — local OLED interface (SSD1306 128x64) + encoder/button control.
 * Runs entirely on core 1 (UI task). Never touches the audio rings; all
 * interaction with the DSP goes through dsp_get_settings()/dsp_set_field()/
 * dsp_apply_settings() and the tuner enable flag.
 */
#ifndef __GTR_UI_H__
#define __GTR_UI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Create + pin the core-1 UI task (after ssd1306/encoder init is done
 * inside the task itself, so ordering against main.c stays forgiving). */
void ui_start_task(void);

#ifdef __cplusplus
}
#endif

#endif
