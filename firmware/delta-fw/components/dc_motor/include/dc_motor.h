#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include <stdbool.h>

typedef struct {
    gpio_num_t pin_pwm;
    gpio_num_t pin_dir;
    ledc_channel_t ch;
    ledc_timer_t timer;
    bool dir_inverted;
    int freq_hz;
    int duty_res_bits; // e.g., 12
} dc_motor_t;

// Initialize one DC motor (PWM on pin_pwm, logic DIR on pin_dir)
bool dc_motor_init(dc_motor_t* m);

// Set signed speed in [-1, 1] (chassis). Clamps internally.
void dc_motor_set_signed(dc_motor_t* m, float speed);

// Set duty in [0, 1] with dir=false (pump run forward only).
void dc_motor_set_forward(dc_motor_t* m, float duty);
