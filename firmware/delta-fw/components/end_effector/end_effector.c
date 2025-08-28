#include "end_effector.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dc_motor.h"

static const char *EE_TAG = "end_effector";
static volatile bool ee_busy = false;

// Default pump wiring (can be adjusted here or via future config API)
static const gpio_num_t PUMP_PWM = GPIO_NUM_19;
static const gpio_num_t PUMP_DIR = GPIO_NUM_2;    // change if strapping pin conflicts
static dc_motor_t pump = {
    .pin_pwm = PUMP_PWM, .pin_dir = PUMP_DIR,
    .ch = LEDC_CHANNEL_1, .timer = LEDC_TIMER_0,
    .dir_inverted = false, .freq_hz = 20000, .duty_res_bits = 12
};

void end_effector_init(void) {
    ESP_LOGI(EE_TAG, "end_effector_init: skeleton initialized");
    // Initialize pump driver
    dc_motor_init(&pump);
    // TODO: Configure GPIOs / peripherals for gripper, etc.
}

void end_effector_task(void *arg) {
    (void)arg;
    ESP_LOGI(EE_TAG, "end_effector_task started");
    for (;;) {
        // TODO: implement periodic monitoring/control
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void end_effector_open(void) {
    ee_busy = true;
    ESP_LOGI(EE_TAG, "end_effector_open called (skeleton)");
    // TODO: drive hardware
    ee_busy = false;
}

void end_effector_close(void) {
    ee_busy = true;
    ESP_LOGI(EE_TAG, "end_effector_close called (skeleton)");
    // TODO: drive hardware
    ee_busy = false;
}

bool end_effector_is_busy(void) {
    return ee_busy;
}

void end_effector_set_pump(float duty01) {
    if (duty01 < 0.0f) duty01 = 0.0f;
    if (duty01 > 1.0f) duty01 = 1.0f;
    dc_motor_set_forward(&pump, duty01);
}
