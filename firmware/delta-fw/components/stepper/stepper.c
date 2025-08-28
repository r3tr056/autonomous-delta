#include "stepper.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

struct stepper_t {
    stepper_pins_t pins;            // user supplied PIN MAP
    uint32_t cur_int_us;            // current step interval
    uint32_t tgt_int_us;            // target step interval (speed setpoint)
    uint32_t accel_us_per_step;     // delta interval per step (ramps)
    bool ramping;                   // ramp still in progress ?
};


stepper_t* stepper_create(const stepper_pins_t* pins)
{
    if (!pins) return NULL;
    stepper_t* s = (stepper_t*)calloc(1, sizeof(stepper_t));
    if (!s) return NULL;
    s->pins = *pins;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << s->pins.pin_pul) | (1ULL << s->pins.pin_dir),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE
    };
    if (s->pins.pin_ena >= 0) {
        io.pin_bit_mask |= (1ULL << s->pins.pin_ena);
    }
    gpio_config(&io);

    // Defaults
    gpio_set_level(s->pins.pin_pul, 0);
    gpio_set_level(s->pins.pin_dir, s->pins.dir_inverted ? 1 : 0);
    if (s->pins.pin_ena >= 0) {
        gpio_set_level(s->pins.pin_ena, s->pins.ena_inverted ? 1 : 0);
    }

    /* default 500Hz until user changes it */
    s->cur_int_us = s->tgt_int_us = 2000;
    return s;
}

void stepper_destroy(stepper_t* s) {
    if (!s) return;
    free(s);
}

void stepper_enable(stepper_t* s, bool enable) {
    if (!s || s->pins.pin_ena < 0) return;
    // ENA low-active is common on TB6600: ena_inverted=true will flip logic
    gpio_set_level(s->pins.pin_ena, enable ? (s->pins.ena_inverted ? 0 : 1) : (s->pins.ena_inverted ? 1 : 0));
}

/* DIR changes need >=2.2us setup before first pulse */
void stepper_set_dir(stepper_t* s, bool dir) {
    if (!s) return;
    gpio_set_level(s->pins.pin_dir, dir ^ s->pins.dir_inverted);
    esp_rom_delay_us(5); // TB6600 DIR-setup delay
}

/* interval_is = step period (us). accel_us_per_step controls ramp slope.
accel 0 -> immediate speed change. */
void stepper_set_speed(stepper_t *s, uint32_t interval_us, uint32_t accel_us_per_step) {
    if (!s || interval_us < 4) return;
    s->tgt_int_us = interval_us;
    s->accel_us_per_step = accel_us_per_step;
    s->ramping = accel_us_per_step != 0;
}

/* Pulse generator */
void IRAM_ATTR stepper_step_blocking(stepper_t* s, uint32_t steps, uint32_t pulse_us, uint32_t interval_us)
{
    if (!s || steps == 0) return;
    if (pulse_us < 2) pulse_us = 2;

    for (uint32_t i = 0; i < steps; ++i) {
        /* ramp controller */
        if (s->ramping) {
            if (s->cur_int_us < s->tgt_int_us) {
                s->cur_int_us += s->accel_us_per_step;
                if (s->cur_int_us >= s->tgt_int_us) s->ramping = false;
            } else if (s->cur_int_us > s->tgt_int_us) {
                s->cur_int_us -= s->accel_us_per_step;
                if (s->cur_int_us <= s->tgt_int_us) s->ramping = false;
            } else {
                s->ramping = false;
            }
        }

        /* generate step pulse */
        gpio_set_level(s->pins.pin_pul, 1);
        esp_rom_delay_us(pulse_us);
        gpio_set_level(s->pins.pin_pul, 0);

        uint32_t low_time = (s->cur_int_us > pulse_us) ? s->cur_int_us - pulse_us : 2;
        esp_rom_delay_us(low_time);
    }
}
