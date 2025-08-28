#include "dc_motor.h"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "dc_motor";

bool dc_motor_init(dc_motor_t* m) {
    if (!m) return false;
    // DIR
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << m->pin_dir),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    gpio_set_level(m->pin_dir, m->dir_inverted ? 1 : 0);

    // PWM
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .timer_num = m->timer,
        .duty_resolution = m->duty_res_bits,
        .freq_hz = m->freq_hz,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_timer_config(&tcfg));

    ledc_channel_config_t ccfg = {
        .gpio_num = m->pin_pwm,
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .channel = m->ch,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = m->timer,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_channel_config(&ccfg));
    ESP_LOGI(TAG, "DC motor initialized on PWM=%d DIR=%d @%dHz res=%d", (int)m->pin_pwm, (int)m->pin_dir, m->freq_hz, m->duty_res_bits);
    return true;
}

static inline void set_dir(dc_motor_t* m, bool forward) {
    int level = forward ^ m->dir_inverted ? 1 : 0;
    gpio_set_level(m->pin_dir, level);
}

static inline void set_pwm(dc_motor_t* m, float duty01) {
    if (duty01 < 0) duty01 = 0;
    if (duty01 > 1) duty01 = 1;
    uint32_t maxd = (1u << m->duty_res_bits) - 1;
    uint32_t d = (uint32_t)lroundf(duty01 * (float)maxd);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, m->ch, d);
    ledc_update_duty(LEDC_SPEED_MODE_MAX, m->ch);
}

void dc_motor_set_signed(dc_motor_t* m, float speed) {
    if (!m) return;
    bool forward = speed >= 0.0f;
    float mag = fabsf(speed);
    set_dir(m, forward);
    set_pwm(m, mag);
}

void dc_motor_set_forward(dc_motor_t* m, float duty) {
    if (!m) return;
    set_dir(m, true);
    set_pwm(m, duty);
}
