/*
 * battery.c — ADC1 continuous read of GPIO2 (ADC1_CH1).
 * The ESP32-S3 ADC has noticeable non-linearity at the very bottom/top of the
 * range, so we use the calibrated curve fitting + a 9-sample median.
 */
#include "battery.h"
#include "board.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <stdlib.h>

static const char *TAG = "battery";

static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t cali;
static bool cali_ok;
static uint16_t last_mv;
static uint8_t  last_pct;

void battery_init(void)
{
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &adc));

    adc_oneshot_chan_cfg_t ccfg = {
        .atten = ADC_ATTEN_DB_12,          /* full-range capable (~3.1 V pin max before div) */
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, BAT_ADC_CHANNEL, &ccfg));

    adc_cali_curve_fitting_config_t cfcfg = {
        .unit_id = ADC_UNIT_1, .chan = BAT_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    cali_ok = (adc_cali_create_scheme_curve_fitting(&cfcfg, &cali) == ESP_OK);
    if (!cali_ok)
        ESP_LOGW(TAG, "no ADC calibration data; using raw linear conversion");
    last_mv = 0; last_pct = 0;
}

static int pin_uv(int raw)
{
    if (cali_ok) { int uv = 0; adc_cali_raw_to_voltage(cali, raw, &uv); return uv; }
    return raw * 3100 * 1000 / 4095;      /* rough fallback for DB_12 */
}

static int cmp_u16(const void *a, const void *b)
{ return *(const uint16_t *)a - *(const uint16_t *)b; }

void battery_sample(void)
{
    uint16_t samples[9];
    for (int i = 0; i < 9; i++) {
        int raw = 0;
        adc_oneshot_read(adc, BAT_ADC_CHANNEL, &raw);
        samples[i] = (uint16_t)pin_uv(raw) / 1000;   /* mV at the pin */
    }
    qsort(samples, 9, sizeof(samples[0]), cmp_u16);
    uint32_t mv = (uint32_t)samples[4] * BAT_DIVIDER_X100 / 100;  /* divider */
    last_mv = (uint16_t)mv;

    int span = BAT_FULL_MV - BAT_EMPTY_MV;
    int p = ((int)mv - BAT_EMPTY_MV) * 100 / (span > 0 ? span : 1);
    if (p < 0) p = 0;
    if (p > 100) p = 100;
    last_pct = (uint8_t)p;
}

uint16_t battery_mv(void)  { return last_mv; }
uint8_t  battery_pct(void) { return last_pct; }
bool     battery_low(void) { return last_pct < 20; }
