/*
 * ssd1306.c — SSD1306 driver using the esp_driver_i2c master bus.
 * 128x64, page addressing, internal 1 KB framebuffer.
 */
#include <string.h>
#include <stdio.h>
#include "ssd1306.h"
#include "board.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "ssd1306";

#define OLED_W 128
#define OLED_H 64
#define PAGES  (OLED_H / 8)

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t dev;
static uint8_t fb[PAGES][OLED_W];

static bool cmd(uint8_t c)
{
    uint8_t b[2] = { 0x00, c };                 /* Co=0, D/C#=0 (command)     */
    return i2c_master_transmit(dev, b, sizeof(b), 50) == ESP_OK;
}

bool ssd1306_init(void)
{
    i2c_master_bus_config_t bc = {
        .i2c_port          = OLED_I2C_PORT,
        .sda_io_num        = OLED_SDA_PIN,
        .scl_io_num        = OLED_SCL_PIN,
        .clk_source        = I2C_SRC_CLK_APB,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bc, &bus) != ESP_OK) return false;

    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = OLED_I2C_ADDR,
        .scl_speed_hz    = 400000,
    };
    if (i2c_master_bus_add_device(bus, &dc, &dev) != ESP_OK) return false;

    static const uint8_t seq[] = {
        0xAE,           /* off            */
        0xD5, 0x80,
        0xA8, 0x3F,     /* multiplex 64   */
        0xD3, 0x00,
        0x40,           /* start line 0   */
        0x8D, 0x14,     /* charge pump on */
        0x20, 0x02,     /* page addressing*/
        0xA1,           /* seg remap      */
        0xC8,           /* com scan dec   */
        0xDA, 0x12,
        0x81, 0x9F,     /* contrast       */
        0xD9, 0x1F,
        0xDB, 0x40,
        0xA4,
        0xA6,           /* non-inverted   */
        0xAF,           /* on             */
    };
    for (unsigned i = 0; i < sizeof(seq); i++)
        if (!cmd(seq[i])) { ESP_LOGE(TAG, "I2C init fail"); return false; }

    ssd1306_clear();
    return true;
}

void ssd1306_clear(void)
{
    memset(fb, 0, sizeof(fb));
    ssd1306_flush();
}

void ssd1306_flush(void)
{
    /* one DMA buffer reused: [0x40 header | 128 px] per page */
    _Static_assert(sizeof(fb) + 1 <= 1024, "oled txbuf");
    static uint8_t txbuf[sizeof(fb) + 1];
    txbuf[0] = 0x40;
    memcpy(txbuf + 1, fb, sizeof(fb));

    for (int p = 0; p < PAGES; p++) {
        cmd(0xB0 | p);
        cmd(0x00 | (p & 0x0F));         /* lower col (page mode wraps)      */
        cmd(0x10);                      /* upper col = 0                    */
        uint8_t b[1 + OLED_W];
        b[0] = 0x40;
        memcpy(b + 1, fb[p], OLED_W);
        if (i2c_master_transmit(dev, b, sizeof(b), 100) != ESP_OK) return;
    }
}

void oled_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
    if (on) fb[y >> 3][x] |=  (1 << (y & 7));
    else    fb[y >> 3][x] &= ~(1 << (y & 7));
}

void oled_hline(int x, int y, int w, bool on)
{ for (int i = 0; i < w; i++) oled_pixel(x + i, y, on); }

void oled_rect(int x, int y, int w, int h, bool on)
{
    oled_hline(x, y, w, on);
    oled_hline(x, y + h - 1, w, on);
    for (int j = 0; j < h; j++) { oled_pixel(x, y + j, on); oled_pixel(x + w - 1, y + j, on); }
}

void oled_fill_rect(int x, int y, int w, int h, bool on)
{ for (int j = 0; j < h; j++) oled_hline(x, y + j, w, on); }

/* ---------------- 6x8 font (5 columns + spacing) ----------------
 * Standard classic ASCII cell, rows LSB=top. */
static const uint8_t *glyph(unsigned char c)
{
    /* compact table for the character set this UI actually prints */
    static const uint8_t F[] = {
        /* ' ' */ 0x00,0x00,0x00,0x00,0x00,
        /* '!' */ 0x00,0x00,0x5F,0x00,0x00,
        /* '"' */ 0x00,0x07,0x00,0x07,0x00,
        /* '#' */ 0x14,0x7F,0x14,0x7F,0x14,
        /* '%' */ 0x22,0x08,0x08,0x14,0x22,   /* simplified */
        /* '&' */ 0x36,0x49,0x56,0x20,0x50,
        /* '\''*/ 0x00,0x05,0x03,0x00,0x00,
        /* '(' */ 0x00,0x1C,0x22,0x41,0x00,
        /* ')' */ 0x00,0x41,0x22,0x1C,0x00,
        /* '*' */ 0x14,0x08,0x3E,0x08,0x14,
        /* '+' */ 0x08,0x08,0x3E,0x08,0x08,
        /* ',' */ 0x00,0x50,0x30,0x00,0x00,
        /* '-' */ 0x08,0x08,0x08,0x08,0x08,
        /* '.' */ 0x00,0x60,0x60,0x00,0x00,
        /* '/' */ 0x20,0x10,0x08,0x04,0x02,
    };
    static const uint8_t D0[] = { 0x7F,0x41,0x5D,0x59,0x4F };   /* 0 */
    static const uint8_t DIG[][5] = {
        {0x7F,0x41,0x5D,0x59,0x4F},{0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x36},
        {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},
    };
    (void)F; (void)D0;
    return NULL;
}

/* Full 5x7 ASCII table generated from a public-domain font (see docs/). */
#include "font5x7.inc"

void oled_char(int x, int page, char c, bool inv)
{
    const uint8_t *g = FONT5X7[(unsigned char)c];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 8; row++) {          /* bit7 = baseline gap */
            bool on = (bits >> row) & 1;
            if (row == 7) on = false;
            oled_pixel(x + col, page * 8 + row, inv ? !on : on);
        }
    }
    /* inter-character space */
    for (int row = 0; row < 8; row++)
        oled_pixel(x + 5, page * 8 + row, inv);
}

void oled_str(int x, int page, const char *s, bool inv)
{
    while (*s && x <= OLED_W - 6) {
        oled_char(x, page, *s++, inv);
        x += 6;
    }
}

void oled_bar(int x, int page, int w, int filled, bool on)
{
    (void)on;
    oled_rect(x, page * 8, w, 8, true);
    if (filled > w - 2) filled = w - 2;
    if (filled > 0)
        oled_fill_rect(x + 1, page * 8 + 1, filled, 6, true);
}
