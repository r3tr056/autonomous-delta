#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t pin_pul;
    gpio_num_t pin_dir;
    gpio_num_t pin_ena; // optional, set to -1 if unused
    bool dir_inverted;
    bool ena_inverted;
} stepper_pins_t;

typedef struct stepper_t stepper_t;

// Create and init a stepper instance
stepper_t* stepper_create(const stepper_pins_t* pins);

// Destroy instance
void stepper_destroy(stepper_t* s);

// Enable/disable driver
void stepper_enable(stepper_t* s, bool enable);

// Set direction
void stepper_set_dir(stepper_t* s, bool dir);

// Set speed
void stepper_set_speed(stepper_t *s, uint32_t interval_us, uint32_t accel_us_per_step);

// Generate N steps with given pulse width/us and interval/us (blocking)
void stepper_step_blocking(stepper_t* s, uint32_t steps, uint32_t pulse_us, uint32_t interval_us);

#ifdef __cplusplus
}
#endif
