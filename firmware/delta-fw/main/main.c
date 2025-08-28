#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <std_srvs/srv/set_bool.h>

#include "usb_serial_transport.h"
#include "stepper.h"
#include "dc_motor.h"
#include "end_effector.h"
#include "battery_monitor.h"

static const char *TAG = "delta_firmware";

static const gpio_num_t CHASSIS_PWM = GPIO_NUM_21;
static const gpio_num_t CHASSIS_DIR = GPIO_NUM_18;

static dc_motor_t chassis = {
    .pin_pwm = CHASSIS_PWM, .pin_dir = CHASSIS_DIR,
    .ch = LEDC_CHANNEL_0, .timer = LEDC_TIMER_0,
    .dir_inverted = false, .freq_hz = 20000, .duty_res_bits = 12
};


static const gpio_num_t ESTOP_PIN = GPIO_NUM_4;

// --- LED indicators ---
static const gpio_num_t LED_RED_PIN   = GPIO_NUM_5;
static const gpio_num_t LED_BLUE_PIN  = GPIO_NUM_16;
static const gpio_num_t LED_GREEN_PIN = GPIO_NUM_17;
static const bool LED_ACTIVE_LOW = false;

static inline void led_set(gpio_num_t pin, bool on) {
    gpio_set_level(pin, LED_ACTIVE_LOW ? !on : on);
}

static inline bool read_active_low(gpio_num_t pin) {
    return gpio_get_level(pin) == 0; // using internal pull-ups, closed->GND->0
}

static inline bool estop_pressed(void) { return read_active_low(ESTOP_PIN); }


rcl_publisher_t publisher;           // heartbeat
rcl_publisher_t joint_pub;           // joint positions
rcl_publisher_t battery_pub;         // battery [V, A, SOC]
rcl_subscription_t subscriber;       // string cmds
rcl_subscription_t target_sub;       // joint targets
rcl_subscription_t chassis_sub;      // chassis DC motor cmd
rcl_subscription_t pump_sub;         // pump DC motor cmd
rcl_service_t ee_service;            // end-effector SetBool service
std_msgs__msg__Int32 msg;
std_msgs__msg__String sub_msg;
std_msgs__msg__Float32MultiArray joint_pos_msg;
std_msgs__msg__Float32MultiArray target_msg;
std_msgs__msg__Float32MultiArray battery_msg;
std_msgs__msg__Float32 chassis_cmd_msg;
std_msgs__msg__Float32 pump_cmd_msg;

static float battery_data[3]; // [voltage, current, soc]
static std_srvs__srv__SetBool_Request ee_req;
static std_srvs__srv__SetBool_Response ee_res;
static char ee_res_message_buf[64];


#define SUBSCRIBER_STRING_BUFFER_SIZE 128
static char subscriber_string_buffer[SUBSCRIBER_STRING_BUFFER_SIZE];

static float joint_pos_data[3];
static float target_data_buf[3];

static const float STEPS_PER_REV = 200.0f;       // NEMA 23 typical 1.8°/step -> 200 full steps/rev
static const float MICROSTEP     = 16.0f;        // TB6600 microstep setting (match hardware DIP switches)
static const float GEAR_RATIO    = 1.0f;         // belt/gear reduction if any
static const float STEPS_PER_RAD = (STEPS_PER_REV * MICROSTEP * GEAR_RATIO) / (2.0f * 3.14159265358979323846f);

static const uint32_t MAX_SPS = 4000;           // max steps/sec per axis - higher for delta precision
static const uint32_t PULSE_US = 5;             // TB6600 min 2.2us; use 5us for high speed operation
static const uint32_t ACCEL_STEPS = 50;         // steps for acceleration/deceleration

static const float JOINT_MIN[3] = { -2.0944f, -2.0944f, -2.0944f }; // -120 deg for delta range
static const float JOINT_MAX[3] = {  2.0944f,  2.0944f,  2.0944f }; // +120 deg for delta range

static stepper_t* steppers[3] = {0};

typedef struct {
    volatile int32_t pos_steps;      // current position in steps
    volatile int32_t target_steps;   // target position in steps
    volatile int32_t vel_steps_s;    // current velocity in steps/sec
    volatile bool enabled;           // axis enabled state
    volatile bool moving;            // axis currently moving
    bool dir_positive;               // current direction
    uint32_t last_step_time_us;      // last step timestamp for velocity control
} axis_state_t;
static axis_state_t axis[3] = {0};

static volatile bool estop_active = false;

static void chassis_callback(const void * msgin) {
    const std_msgs__msg__Float32* m = (const std_msgs__msg__Float32*)msgin;
    if (estop_active) { dc_motor_set_signed(&chassis, 0.0f); return; }
    float cmd = m->data; if (cmd < -1.0f) cmd = -1.0f; if (cmd > 1.0f) cmd = 1.0f;
    dc_motor_set_signed(&chassis, cmd);
}

static void pump_callback(const void * msgin) {
    const std_msgs__msg__Float32* m = (const std_msgs__msg__Float32*)msgin;
    if (estop_active) { end_effector_set_pump(0.0f); return; }
    float duty = m->data; if (duty < 0.0f) duty = 0.0f; if (duty > 1.0f) duty = 1.0f;
    end_effector_set_pump(duty);
}
static volatile bool coordinated_move = false;

typedef struct {
    int axis;           // 0..2
    int steps;          // +/- steps for direction
    uint32_t pulse_us;  // pulse high width
    uint32_t period_us; // full period
    bool pending;
} jog_cmd_t;
static jog_cmd_t jog_cmd = { .pending = false };

static inline int32_t rad_to_steps(float rad);
static inline int32_t clamp_steps_to_limits(int axis_idx, int32_t desired_steps);

static void jog_task(void* arg) {
    (void)arg;

    for (;;) {
        if (jog_cmd.pending) {
            jog_cmd_t cmd = jog_cmd;
            jog_cmd.pending = false;

            int ax = cmd.axis;
            int delta = cmd.steps;
            if (estop_active) {
                ESP_LOGW(TAG, "Jog ignored: ESTOP active");
            } else if (ax < 0 || ax >= 3 || steppers[ax] == NULL) {
                ESP_LOGW(TAG, "Jog ignored: invalid axis %d", ax);
            } else if (!axis[ax].enabled) {
                ESP_LOGW(TAG, "Jog ignored: axis %d disabled", ax);
            } else if (delta == 0) {
                // no-op
            } else {
                int32_t desired = axis[ax].target_steps + delta;
                // Apply soft limits
                desired = clamp_steps_to_limits(ax, desired);

                // No home switch guard; rely on software limits only

                // Set new target; axis_task will execute motion at configured rate
                axis[ax].target_steps = desired;
                ESP_LOGI(TAG, "Jog applied: axis %d delta %d -> target %ld (pulse %u, period %u ignored; using MAX_SPS)",
                         ax, delta, (long)desired, (unsigned)cmd.pulse_us, (unsigned)cmd.period_us);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void safety_task(void* arg) {
    (void)arg;
    bool last_estop = false;
    
    for (;;) {
        bool e = estop_pressed();
        
        // Delta safety: immediate stop on E-Stop activation
        if (e && !last_estop) {
            estop_active = true;
            ESP_LOGW(TAG, "EMERGENCY STOP ACTIVATED - All axes disabled");
            
            // Immediate disable all drivers for delta safety
            for (int i = 0; i < 3; ++i) {
                if (steppers[i]) {
                    stepper_enable(steppers[i], false);
                }
                axis[i].enabled = false;
                axis[i].moving = false;
                axis[i].vel_steps_s = 0;
            }
            coordinated_move = false;
        }
        last_estop = e;
        
    // Delta robot specific: monitor all axes for coordinated motion safety
        if (!estop_active) {
            bool any_moving = false;
            for (int i = 0; i < 3; ++i) {
                if (axis[i].moving) {
                    any_moving = true;
                    break;
                }
            }
            coordinated_move = any_moving;
        }

        
        vTaskDelay(pdMS_TO_TICKS(2)); // 500 Hz for delta safety monitoring
    }
}

static void status_task(void* arg) {
    (void)arg;
    // Red LED indicates power: keep ON continuously
    led_set(LED_RED_PIN, true);
    bool last_blue = false;
    bool last_green = false;
    for (;;) {
        // Blue LED: end-effector activity
        bool busy = end_effector_is_busy();
        if (busy != last_blue) {
            led_set(LED_BLUE_PIN, busy);
            last_blue = busy;
        }
        // Green LED: ROS2 connectivity to agent
        bool connected = (rmw_uros_ping_agent(10, 1) == RMW_RET_OK);
        if (connected != last_green) {
            led_set(LED_GREEN_PIN, connected);
            last_green = connected;
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // 5 Hz updates
    }
}

static inline int32_t rad_to_steps(float rad) {
    return (int32_t)(rad * STEPS_PER_RAD);
}

static inline float steps_to_rad(int32_t steps) {
    return ((float)steps) / STEPS_PER_RAD;
}

static inline int32_t clamp_steps_to_limits(int axis_idx, int32_t desired_steps) {
    int32_t min_s = rad_to_steps(JOINT_MIN[axis_idx]);
    int32_t max_s = rad_to_steps(JOINT_MAX[axis_idx]);
    if (desired_steps < min_s) return min_s;
    if (desired_steps > max_s) return max_s;
    return desired_steps;
}

static void axis_task(void* arg) {
    int idx = (int)(intptr_t)arg;
    const uint32_t min_period_us = (MAX_SPS > 0) ? (1000000UL / MAX_SPS) : 250; // high performance timing
    
    // Delta robot optimization: stagger axis startup to reduce simultaneous current draw
    vTaskDelay(pdMS_TO_TICKS(idx * 5));

    for (;;) {
        if (estop_active) { 
            axis[idx].moving = false;
            vTaskDelay(pdMS_TO_TICKS(10)); 
            continue; 
        }
        if (!steppers[idx] || !axis[idx].enabled) { 
            axis[idx].moving = false;
            vTaskDelay(pdMS_TO_TICKS(10)); 
            continue; 
        }

        int32_t current = axis[idx].pos_steps;
        int32_t target  = axis[idx].target_steps;
        int32_t error = target - current;
        
        if (error == 0) { 
            axis[idx].moving = false;
            axis[idx].vel_steps_s = 0;
            vTaskDelay(pdMS_TO_TICKS(1)); // faster response for delta precision
            continue; 
        }

        axis[idx].moving = true;
        bool dir = (error > 0);
        
        stepper_set_dir(steppers[idx], dir);
        axis[idx].dir_positive = dir;

        // Variable timing based on error magnitude for smoother motion
        uint32_t period_us = min_period_us;
        int32_t abs_error = (error > 0) ? error : -error;
        if (abs_error < ACCEL_STEPS) {
            period_us = min_period_us + (min_period_us * (ACCEL_STEPS - abs_error)) / ACCEL_STEPS;
        }

        // Program speed (driver ramps to target interval internally)
        stepper_set_speed(steppers[idx], period_us, 5);
        // Execute exactly one step using current interval
        stepper_step_blocking(steppers[idx], 1, PULSE_US, 0);

        // Update position and velocity
        uint32_t now_us = esp_timer_get_time();
        if (dir) { 
            axis[idx].pos_steps++; 
        } else { 
            axis[idx].pos_steps--; 
        }
        
        // Calculate velocity for feedback
        if (axis[idx].last_step_time_us > 0) {
            uint32_t dt_us = now_us - axis[idx].last_step_time_us;
            if (dt_us > 0) {
                axis[idx].vel_steps_s = 1000000UL / dt_us;
            }
        }
        axis[idx].last_step_time_us = now_us;

        // Enforce absolute limits (critical for delta robot safety)
        int32_t min_s = rad_to_steps(JOINT_MIN[idx]);
        int32_t max_s = rad_to_steps(JOINT_MAX[idx]);
        if (axis[idx].pos_steps < min_s) { 
            axis[idx].pos_steps = min_s; 
            axis[idx].target_steps = min_s; 
            ESP_LOGW(TAG, "Axis %d hit min limit", idx);
        }
        else if (axis[idx].pos_steps > max_s) { 
            axis[idx].pos_steps = max_s; 
            axis[idx].target_steps = max_s; 
            ESP_LOGW(TAG, "Axis %d hit max limit", idx);
        }
    }
}

static void target_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray* m = (const std_msgs__msg__Float32MultiArray*)msgin;
    
    if (estop_active) { 
        ESP_LOGW(TAG, "Ignoring targets: ESTOP active"); 
        return; 
    }
    
    // Delta robot: expect exactly 3 joint angles from RaspberryPi kinematics
    size_t n = m->data.size;
    if (n != 3) {
        ESP_LOGW(TAG, "Delta target: expected 3 joints, got %u", (unsigned)n);
        return;
    }
    
    // Delta optimization: validate all targets before applying any
    int32_t new_targets[3];
    bool valid = true;
    
    for (int i = 0; i < 3; ++i) {
        float rad = m->data.data[i];
        
        // Clamp to delta joint limits
        if (rad < JOINT_MIN[i]) rad = JOINT_MIN[i];
        if (rad > JOINT_MAX[i]) rad = JOINT_MAX[i];
        
        new_targets[i] = rad_to_steps(rad);
        new_targets[i] = clamp_steps_to_limits(i, new_targets[i]);
        
        // Delta safety: check for reasonable motion increments
        int32_t delta = new_targets[i] - axis[i].target_steps;
        int32_t abs_delta = (delta > 0) ? delta : -delta;
        
        // Warn for large jumps that might indicate kinematic errors
        if (abs_delta > rad_to_steps(0.5)) { // > 0.5 rad jump
            ESP_LOGW(TAG, "Large delta motion axis %d: %ld steps (%.2f rad)", 
                     i, (long)abs_delta, (float)abs_delta / STEPS_PER_RAD);
        }
    }
    
    // Apply all targets atomically for coordinated delta motion
    if (valid) {
        for (int i = 0; i < 3; ++i) {
            axis[i].target_steps = new_targets[i];
        }
        coordinated_move = true;
    }
}

static void ee_service_callback(const void * req, void * res)
{
    const std_srvs__srv__SetBool_Request * request = (const std_srvs__srv__SetBool_Request *)req;
    std_srvs__srv__SetBool_Response * response = (std_srvs__srv__SetBool_Response *)res;

    // Semantics: data=true -> close; data=false -> open
    bool close_cmd = request->data;
    if (close_cmd) {
        end_effector_close();
        response->success = true;
        const char * msg = "gripper closed";
        size_t len = strlen(msg);
        if (response->message.capacity > len) {
            memcpy(response->message.data, msg, len + 1);
            response->message.size = len;
        } else {
            response->message.size = 0; // buffer too small (should not happen with default allocator)
        }
    } else {
        end_effector_open();
        response->success = true;
        const char * msg = "gripper opened";
        size_t len = strlen(msg);
        if (response->message.capacity > len) {
            memcpy(response->message.data, msg, len + 1);
            response->message.size = len;
        } else {
            response->message.size = 0;
        }
    }
}

void subscription_callback(const void * msgin)
{
    const std_msgs__msg__String * m = (const std_msgs__msg__String *)msgin;
    ESP_LOGI(TAG, "Received: %s", m->data.data);

    // Minimalistic text protocol (space-separated):
    // enable <0|1>
    // estop_clear
    // estop_status
    // soft_zero
    // dir <axis 0-2> <0|1>
    // jog <axis 0-2> <steps +/-int> <pulse_us> <period_us>
    if (strncmp(m->data.data, "enable ", 7) == 0) {
        int en = atoi(&m->data.data[7]);
        if (en && estop_active) {
            ESP_LOGW(TAG, "Cannot enable: ESTOP active. Send estop_clear after releasing E-Stop.");
            return;
        }
        for (int i = 0; i < 3; ++i) {
            if (steppers[i]) {
                stepper_enable(steppers[i], en != 0);
                axis[i].enabled = (en != 0) && !estop_active;
            }
        }
        ESP_LOGI(TAG, "Enable set to %d", en);
    } else if (strncmp(m->data.data, "estop_clear", 11) == 0) {
        if (!estop_pressed()) {
            estop_active = false;
            ESP_LOGW(TAG, "ESTOP cleared. Re-enable drivers with 'enable 1'.");
        } else {
            ESP_LOGW(TAG, "Cannot clear: E-Stop input still asserted.");
        }
    } else if (strncmp(m->data.data, "estop_status", 12) == 0) {
        ESP_LOGI(TAG, "ESTOP %s (input=%d)", estop_active ? "ACTIVE" : "OK", estop_pressed());
    } else if (strncmp(m->data.data, "soft_zero", 9) == 0) {
        for (int i = 0; i < 3; ++i) { axis[i].pos_steps = 0; axis[i].target_steps = 0; }
        ESP_LOGI(TAG, "Software zeroed all axes");
    } else if (strncmp(m->data.data, "dir ", 4) == 0) {
        int ax = -1, dir = 0;
        if (sscanf(&m->data.data[4], "%d %d", &ax, &dir) == 2) {
            if (ax >= 0 && ax < 3 && steppers[ax]) {
                stepper_set_dir(steppers[ax], dir != 0);
                ESP_LOGI(TAG, "Axis %d dir %d", ax, dir);
            }
        }
    } else if (strncmp(m->data.data, "jog ", 4) == 0) {
        int ax = -1, steps = 0;
        unsigned pulse = PULSE_US, period = (MAX_SPS ? (1000000 / MAX_SPS) : 500);
        if (sscanf(&m->data.data[4], "%d %d %u %u", &ax, &steps, &pulse, &period) >= 2) {
            if (ax >= 0 && ax < 3 && steppers[ax]) {
                jog_cmd.axis = ax;
                jog_cmd.steps = steps;
                jog_cmd.pulse_us = pulse;
                jog_cmd.period_us = period;
                jog_cmd.pending = true;
                ESP_LOGI(TAG, "Jog axis %d steps %d pulse %u period %u", ax, steps, pulse, period);
            }
        }
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Heartbeat with system status
        msg.data++;
        rcl_ret_t ret1 = rcl_publish(&publisher, &msg, NULL);
        if (ret1 != RCL_RET_OK) {
            ESP_LOGE(TAG, "Error publishing heartbeat: %s", rcl_get_error_string().str);
        }

        // Joint positions for delta feedback to RaspberryPi
        for (int i = 0; i < 3; ++i) {
            joint_pos_data[i] = steps_to_rad(axis[i].pos_steps);
        }
        joint_pos_msg.data.size = 3;
        rcl_ret_t ret2 = rcl_publish(&joint_pub, &joint_pos_msg, NULL);
        if (ret2 != RCL_RET_OK) {
            ESP_LOGE(TAG, "Error publishing joint positions: %s", rcl_get_error_string().str);
        }

        // Battery monitoring (1 Hz rate)
        static uint32_t bat_tick = 0;
        if (++bat_tick >= 20) { // timer is 50 ms -> 20 ticks ~ 1 s
            bat_tick = 0;
            battery_data[0] = battery_get_voltage();
            battery_data[1] = battery_get_current();
            battery_data[2] = battery_get_soc();
            battery_msg.data.size = 3;
            rcl_ret_t ret3 = rcl_publish(&battery_pub, &battery_msg, NULL);
            if (ret3 != RCL_RET_OK) {
                ESP_LOGE(TAG, "Error publishing battery: %s", rcl_get_error_string().str);
            }
        }
        
        // Delta status logging (reduce frequency to avoid spam)
        static uint32_t status_tick = 0;
        if (++status_tick >= 200) { // every 10 seconds
            status_tick = 0;
            bool any_moving = false;
            for (int i = 0; i < 3; ++i) {
                if (axis[i].moving) any_moving = true;
            }
            ESP_LOGI(TAG, "Delta status: E-stop=%d, Moving=%d, Pos=[%.3f,%.3f,%.3f]", 
                     estop_active, any_moving,
                     steps_to_rad(axis[0].pos_steps),
                     steps_to_rad(axis[1].pos_steps), 
                     steps_to_rad(axis[2].pos_steps));
        }
    }
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_ret_t ret;

    // Wait for micro-ROS agent to be available (non-fatal if it isn't yet)
    ESP_LOGI(TAG, "Waiting for micro-ROS agent...");
    int attempts = 0;
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK && attempts < 50) {
        vTaskDelay(pdMS_TO_TICKS(100));
        attempts++;
    }
    if (attempts >= 50) {
        ESP_LOGW(TAG, "micro-ROS agent not reachable, continuing initialization");
    } else {
        ESP_LOGI(TAG, "micro-ROS agent reachable");
    }

    // Initialize micro-ROS allocator
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    ret = rcl_init_options_init(&init_options, allocator);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init options");
        vTaskDelete(NULL);
        return;
    }
    
    ret = rcl_init_options_set_domain_id(&init_options, 0);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to set domain ID");
        vTaskDelete(NULL);
        return;
    }

    // Initialize rclc support object with custom options
    ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init support");
        vTaskDelete(NULL);
        return;
    }

    // Create node
    rcl_node_t node;
    ret = rclc_node_init_default(&node, "delta_firmware_node", "", &support);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init node");
        vTaskDelete(NULL);
        return;
    }

    // Heartbeat publisher
    ret = rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "delta_heartbeat");
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init heartbeat publisher");
        vTaskDelete(NULL);
        return;
    }

    // Joint position publisher
    // Pre-allocate Float32MultiArray
    memset(&joint_pos_msg, 0, sizeof(joint_pos_msg));
    joint_pos_msg.data.data = joint_pos_data;
    joint_pos_msg.data.capacity = 3;
    joint_pos_msg.data.size = 3;
    joint_pos_msg.layout.dim.size = 0;      // omit layout dims to save memory
    joint_pos_msg.layout.dim.capacity = 0;
    joint_pos_msg.layout.dim.data = NULL;
    joint_pos_msg.layout.data_offset = 0;

    ret = rclc_publisher_init_best_effort(
        &joint_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "delta_joint_position");
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init joint position publisher");
        vTaskDelete(NULL);
        return;
    }

    // Battery publisher
    memset(&battery_msg, 0, sizeof(battery_msg));
    battery_msg.data.data = battery_data;
    battery_msg.data.capacity = 3;
    battery_msg.data.size = 3;
    battery_msg.layout.dim.size = 0;
    battery_msg.layout.dim.capacity = 0;
    battery_msg.layout.dim.data = NULL;
    battery_msg.layout.data_offset = 0;

    ret = rclc_publisher_init_best_effort(
        &battery_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "delta_battery");
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init battery publisher");
        vTaskDelete(NULL);
        return;
    }

    // Pre-allocate memory for string subscription message
    sub_msg.data.data = subscriber_string_buffer;
    sub_msg.data.size = 0;
    sub_msg.data.capacity = SUBSCRIBER_STRING_BUFFER_SIZE;

    // String command subscriber
    ret = rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "delta_command");
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init subscriber");
        vTaskDelete(NULL);
        return;
    }

    // Target subscriber
    memset(&target_msg, 0, sizeof(target_msg));
    target_msg.data.data = target_data_buf;
    target_msg.data.capacity = 3;
    target_msg.data.size = 0; // will be set by incoming message
    target_msg.layout.dim.size = 0;
    target_msg.layout.dim.capacity = 0;
    target_msg.layout.dim.data = NULL;
    target_msg.layout.data_offset = 0;

    ret = rclc_subscription_init_default(
        &target_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "delta_joint_target");
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init target subscriber");
        vTaskDelete(NULL);
        return;
    }

    // DC motor command subscribers
    ret = rclc_subscription_init_default(
        &chassis_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "chassis_cmd");
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init chassis subscriber");
        vTaskDelete(NULL);
        return;
    }

    ret = rclc_subscription_init_default(
        &pump_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pump_cmd");
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init pump subscriber");
        vTaskDelete(NULL);
        return;
    }

    // End-effector service (std_srvs/SetBool)
    ret = rclc_service_init_default(
        &ee_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
        "end_effector_control");
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init end-effector service");
        vTaskDelete(NULL);
        return;
    }

    // Pre-allocate response string buffer
    memset(&ee_res, 0, sizeof(ee_res));
    ee_res.message.data = ee_res_message_buf;
    ee_res.message.capacity = sizeof(ee_res_message_buf);
    ee_res.message.size = 0;

    // Create timer (20 Hz)
    rcl_timer_t timer;
    const unsigned int timer_timeout = 50; // ms
    ret = rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback,
        true);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init timer");
        vTaskDelete(NULL);
        return;
    }

    // Create executor
    rclc_executor_t executor;
    ret = rclc_executor_init(&executor, &support.context, 4, &allocator);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init executor");
        vTaskDelete(NULL);
        return;
    }
    
    ret = rclc_executor_add_timer(&executor, &timer);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add timer to executor");
        vTaskDelete(NULL);
        return;
    }
    
    ret = rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add subscription to executor");
        vTaskDelete(NULL);
        return;
    }

    ret = rclc_executor_add_subscription(&executor, &target_sub, &target_msg, &target_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add target subscription to executor");
        vTaskDelete(NULL);
        return;
    }

    ret = rclc_executor_add_subscription(&executor, &chassis_sub, &chassis_cmd_msg, &chassis_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add chassis subscription to executor");
        vTaskDelete(NULL);
        return;
    }

    ret = rclc_executor_add_subscription(&executor, &pump_sub, &pump_cmd_msg, &pump_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add pump subscription to executor");
        vTaskDelete(NULL);
        return;
    }

    // Add end-effector service to executor
    ret = rclc_executor_add_service(&executor, &ee_service, &ee_req, &ee_res, ee_service_callback);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add end-effector service to executor");
        vTaskDelete(NULL);
        return;
    }

    // Initialize messages
    msg.data = 0;
    for (int i = 0; i < 3; ++i) {
        joint_pos_data[i] = 0.0f;
    }
    // Battery defaults
    battery_data[0] = battery_get_voltage();
    battery_data[1] = battery_get_current();
    battery_data[2] = battery_get_soc();

    // Spin
    ESP_LOGI(TAG, "micro-ROS initialized successfully, starting main loop");
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Clean up (unreached)
    rcl_ret_t fr;
    fr = rcl_publisher_fini(&publisher, &node); (void)fr;
    fr = rcl_publisher_fini(&joint_pub, &node); (void)fr;
    fr = rcl_publisher_fini(&battery_pub, &node); (void)fr;
    fr = rcl_subscription_fini(&subscriber, &node); (void)fr;
    fr = rcl_subscription_fini(&target_sub, &node); (void)fr;
    fr = rcl_service_fini(&ee_service, &node); (void)fr;
    fr = rcl_node_fini(&node); (void)fr;
    fr = rclc_support_fini(&support); (void)fr;

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Delta firmware starting up...");

    // Configure safety input (E-Stop) with pull-up
    {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << ESTOP_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io);
    }

    // Configure LED outputs
    {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << LED_RED_PIN) | (1ULL << LED_BLUE_PIN) | (1ULL << LED_GREEN_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io);
        led_set(LED_RED_PIN, false);
        led_set(LED_BLUE_PIN, false);
        led_set(LED_GREEN_PIN, false);
    }

    end_effector_init();
    battery_monitor_init();
    dc_motor_init(&chassis);

    // Initialize steppers for delta robot - optimized pin assignment
    // Delta robot convention: Axis 0=A, 1=B, 2=C (120° apart)
    // Using TB6600 drivers with enable active-low
    const stepper_pins_t cfg[3] = {
        { .pin_pul = GPIO_NUM_25, .pin_dir = GPIO_NUM_26, .pin_ena = GPIO_NUM_27, .dir_inverted = false, .ena_inverted = true }, // Axis A
        { .pin_pul = GPIO_NUM_14, .pin_dir = GPIO_NUM_12, .pin_ena = GPIO_NUM_13, .dir_inverted = false, .ena_inverted = true }, // Axis B  
        { .pin_pul = GPIO_NUM_33, .pin_dir = GPIO_NUM_32, .pin_ena = GPIO_NUM_15, .dir_inverted = false, .ena_inverted = true }  // Axis C
    };
    
    for (int i = 0; i < 3; ++i) {
        steppers[i] = stepper_create(&cfg[i]);
        if (steppers[i]) {
            stepper_enable(steppers[i], false); // start disabled for safety
            
            // Initialize delta axis state
            axis[i].pos_steps = 0;
            axis[i].target_steps = 0;
            axis[i].vel_steps_s = 0;
            axis[i].enabled = false;
            axis[i].moving = false;
            axis[i].dir_positive = true;
            axis[i].last_step_time_us = 0;
            
            ESP_LOGI(TAG, "Delta axis %c initialized (PUL=%d, DIR=%d, ENA=%d)", 
                     'A' + i, cfg[i].pin_pul, cfg[i].pin_dir, cfg[i].pin_ena);
        } else {
            ESP_LOGE(TAG, "Failed to create stepper for delta axis %c", 'A' + i);
        }
    }

    // Start delta robot control tasks with optimized priorities
    // Pin to cores where available for consistent timing
#if CONFIG_FREERTOS_UNICORE
    xTaskCreate(axis_task, "delta_axis_A", 4096, (void*)0, 8, NULL);
    xTaskCreate(axis_task, "delta_axis_B", 4096, (void*)1, 8, NULL);
    xTaskCreate(axis_task, "delta_axis_C", 4096, (void*)2, 8, NULL);
    xTaskCreate(safety_task, "delta_safety", 3072, NULL, 9, NULL);
    xTaskCreate(jog_task, "delta_jog", 3072, NULL, 4, NULL);
    xTaskCreate(end_effector_task, "end_effector", 3072, NULL, 3, NULL);
    xTaskCreate(battery_monitor_task, "battery_mon", 3072, NULL, 3, NULL);
    xTaskCreate(status_task, "status_leds", 2048, NULL, 2, NULL);
#else
    xTaskCreatePinnedToCore(axis_task, "delta_axis_A", 4096, (void*)0, 8, NULL, 0);
    xTaskCreatePinnedToCore(axis_task, "delta_axis_B", 4096, (void*)1, 8, NULL, 0);
    xTaskCreatePinnedToCore(axis_task, "delta_axis_C", 4096, (void*)2, 8, NULL, 0);
    xTaskCreatePinnedToCore(safety_task, "delta_safety", 3072, NULL, 9, NULL, 0);
    xTaskCreatePinnedToCore(jog_task, "delta_jog", 3072, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(end_effector_task, "end_effector", 3072, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(battery_monitor_task, "battery_mon", 3072, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(status_task, "status_leds", 2048, NULL, 2, NULL, 1);
#endif

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    // Set up custom USB serial transport for micro-ROS
    rmw_ret_t transport_ret = rmw_uros_set_custom_transport(
        true,           // framing enabled
        NULL,           // no args needed for USB serial
        usb_serial_open,
        usb_serial_close,
        usb_serial_write,
        usb_serial_read
    );
    
    if (transport_ret != RMW_RET_OK) {
        ESP_LOGE(TAG, "Failed to set up custom USB serial transport");
        return;
    }
    
    ESP_LOGI(TAG, "Custom USB serial transport configured successfully");
#else
#error "micro-ROS custom transport not configured. Please check colcon.meta configuration."
#endif // RMW_UXRCE_TRANSPORT_CUSTOM

    ESP_LOGI(TAG, "Creating delta robot micro-ROS communication task...");
    
#if CONFIG_FREERTOS_UNICORE
    xTaskCreate(
        micro_ros_task,
        "delta_micro_ros",
        16384,
        NULL,
        7,
        NULL);
#else
    xTaskCreatePinnedToCore(
        micro_ros_task,
        "delta_micro_ros",
        16384,
        NULL,
        7,
        NULL,
        1);
#endif

    ESP_LOGI(TAG, "Delta robot firmware initialization complete - Ready for operation");
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}