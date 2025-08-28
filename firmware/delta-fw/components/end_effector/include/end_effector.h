#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize end-effector hardware (GPIOs, drivers, etc.)
void end_effector_init(void);

// Periodic task (optional). Call from app or run as its own task.
void end_effector_task(void *arg);

// Example control hooks you can implement later
void end_effector_open(void);
void end_effector_close(void);
bool end_effector_is_busy(void);

// Pump control (part of end-effector)
// duty in [0..1]; 0 stops the pump. Implemented with LEDC PWM + DIR.
void end_effector_set_pump(float duty01);

#ifdef __cplusplus
}
#endif
