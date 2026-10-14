/*
 * encoder.c — quadrature decoder + triple button short/long press detection.
 * Runs in the UI task (core 1). Sampling at 20 ms comfortably resolves guitar
 * pedalboard-speed rotation and gives a clean 40 ms bounce window.
 */
#include "encoder.h"
#include "board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

/* Valid Gray-code transitions for a rotary encoder, indexed by
 * prev_state*4 + cur_state (states from bits A,B). */
static const int8_t STEP[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0,
};

typedef struct { gpio_num_t pin; uint8_t hist; } btn_t;
static btn_t b_enc, b_act, b_pre;
static uint8_t last_ab;
static int acc;                       /* accumulated quadrature steps */
static uint32_t ticks;                /* UI tick counter              */

static inline uint8_t read_hist(btn_t *b)
{
    b->hist = (uint8_t)((b->hist << 1) | (gpio_get_level(b->pin) ? 1 : 0));
    return b->hist;
}

/* stable low == pressed (all inputs use the internal pull-up):
 * the three most recent samples must all read 0. */
static inline bool stable_low(btn_t *b) { return (b->hist & 0x07) == 0x00; }

void encoder_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << ENC_A_PIN) | (1ULL << ENC_B_PIN) |
                        (1ULL << BTN_ENC_PIN) | (1ULL << BTN_ACT_PIN) |
                        (1ULL << BTN_PRESET_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    last_ab = (uint8_t)((gpio_get_level(ENC_A_PIN) ? 2 : 0) |
                        (gpio_get_level(ENC_B_PIN) ? 1 : 0));
    b_enc.pin = BTN_ENC_PIN;  b_act.pin = BTN_ACT_PIN;  b_pre.pin = BTN_PRESET_PIN;
    b_enc.hist = b_act.hist = b_pre.hist = 0xFF;
    acc = 0; ticks = 0;
}

/* Returns: +1 short press, +2 long press, 0 nothing — for one button.
 * `*startp` holds the UI tick at which the current press began (0 = idle). */
static int btn_event(btn_t *b, uint32_t *startp)
{
    read_hist(b);
    bool now_low = stable_low(b);

    if (now_low && *startp == 0) {                 /* press edge */
        *startp = ticks;
        return 0;
    }
    if (now_low) {                                 /* still held */
        return 0;
    }
    if (*startp != 0) {                            /* release after hold */
        uint32_t held = (ticks - *startp) * CONFIG_UI_PERIOD_MS;
        *startp = 0;
        b->hist = 0xFF;                            /* require fresh debounce */
        return held >= LONG_PRESS_MS ? 2 : 1;
    }
    return 0;
}

ui_event_t encoder_poll(void)
{
    ticks++;
    uint8_t ab = (uint8_t)((gpio_get_level(ENC_A_PIN) ? 2 : 0) |
                           (gpio_get_level(ENC_B_PIN) ? 1 : 0));
    acc += STEP[(last_ab << 2) | ab];
    last_ab = ab;

    static uint32_t enc_hold, act_hold, pre_hold;
    int e_enc = btn_event(&b_enc, &enc_hold);
    int e_act = btn_event(&b_act, &act_hold);
    int e_pre = btn_event(&b_pre, &pre_hold);

    if (e_enc) return e_enc == 2 ? EV_ENC_SW_LONG : EV_ENC_SW_SHORT;
    if (e_act) return e_act == 2 ? EV_ACT_LONG   : EV_ACT_SHORT;
    if (e_pre) return e_pre == 2 ? EV_PRESET_LONG: EV_PRESET_SHORT;

    if (acc >= 1) { acc -= 1; return EV_ENC_CW; }
    if (acc <= -1) { acc += 1; return EV_ENC_CCW; }
    return EV_NONE;
}
