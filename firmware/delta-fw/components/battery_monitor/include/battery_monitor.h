#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize battery monitor (ADC, fuel gauge, etc.)
void battery_monitor_init(void);

// Periodic task to read voltage/current/state
void battery_monitor_task(void *arg);

// Simple getters (populate as you implement)
float battery_get_voltage(void);
float battery_get_current(void);
float battery_get_soc(void); // state of charge 0..100

#ifdef __cplusplus
}
#endif
