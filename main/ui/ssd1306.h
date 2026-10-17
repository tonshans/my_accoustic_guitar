/*
 * ssd1306.h — minimal SSD1306 128x64 OLED driver (software I2C master bus).
 */
#ifndef __SSD1306_H__
#define __SSD1306_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool ssd1306_init(void);
void ssd1306_clear(void);                 /* clears framebuffer + display   */
void ssd1306_flush(void);                 /* push framebuffer over I2C      */

/* Framebuffer drawing helpers. x in pixels, y in 8-pixel pages (0..7). */
void oled_pixel(int x, int y, bool on);
void oled_hline(int x, int y, int w, bool on);
void oled_vline(int x, int y, int h, bool on);
void oled_rect(int x, int y, int w, int h, bool on);          /* outline     */
void oled_fill_rect(int x, int y, int w, int h, bool on);
void oled_char(int x, int page, char c, bool inv);            /* 6x8 font    */
void oled_str(int x, int page, const char *s, bool inv);
void oled_bar(int x, int page, int w, int filled, bool on);   /* level meter */

#ifdef __cplusplus
}
#endif

#endif
