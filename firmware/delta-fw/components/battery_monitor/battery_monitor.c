#include "battery_monitor.h"
// New ADC API (oneshot + calibration)
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define TAG               "battery"

// ADC pins: using ADC1 CH4 (GPIO32), CH5 (GPIO33)
#define V_ADC_CHANNEL     ADC_CHANNEL_4
#define I_ADC_CHANNEL     ADC_CHANNEL_5
#define ADC_UNIT_ID       ADC_UNIT_1
#define ADC_ATTEN         ADC_ATTEN_DB_12 // DB_11 is deprecated; DB_12 has same behavior

/* --- calibration constants (edit if you calibrate) ---------------------- */
#define V_DIVIDER         10.1f           // ratio (R1+R2)/R2
#define I_SHUNT_MV_PER_A  18.182f         // mV per Amp from datasheet
#define BAT_CAPACITY_AH   5.0f            // 5 Ah pack, used for SOC
/* ------------------------------------------------------------------------ */

static adc_oneshot_unit_handle_t s_adc_unit = NULL;
static adc_cali_handle_t s_cali_handle = NULL;
static float g_voltage = 0.0f;   /* V  */
static float g_current = 0.0f;   /* A  */
static float g_soc     = 100.0f; /* %  */
static float coulomb_cnt = BAT_CAPACITY_AH * 3600.0f; /* C */

static int read_channel_mv(adc_channel_t ch)
{
    if (!s_adc_unit) return -1;
    int raw = 0;
    if (adc_oneshot_read(s_adc_unit, ch, &raw) != ESP_OK) return -1;
    int mv = raw;
    if (s_cali_handle) {
        int out_mv = 0;
        if (adc_cali_raw_to_voltage(s_cali_handle, raw, &out_mv) == ESP_OK) mv = out_mv;
    }
    return mv;
}

void battery_monitor_init(void)
{
    // Create ADC oneshot unit
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_ID,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_unit));

    // Configure channels
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_unit, V_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_unit, I_ADC_CHANNEL, &chan_cfg));

    // Try to enable calibration (prefer scheme per Kconfig)
    bool cal_ok = false;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!cal_ok) {
        adc_cali_curve_fitting_config_t cal_cf = {
            .unit_id = ADC_UNIT_ID,
            .chan = V_ADC_CHANNEL, // channel doesn't matter for calibration handle creation
            .atten = ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_curve_fitting(&cal_cf, &s_cali_handle) == ESP_OK) {
            cal_ok = true;
        }
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!cal_ok) {
        adc_cali_line_fitting_config_t cal_lf = {
            .unit_id = ADC_UNIT_ID,
            .atten = ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .default_vref = 1100,
        };
        if (adc_cali_create_scheme_line_fitting(&cal_lf, &s_cali_handle) == ESP_OK) {
            cal_ok = true;
        }
    }
#endif
    ESP_LOGI(TAG, "battery monitor ready, cal=%s", cal_ok ? "on" : "off");
}

void battery_monitor_task(void *arg)
{
    const TickType_t T = pdMS_TO_TICKS(100);   /* 100 ms sample */
    for (;;) {
        /* --- sample ----------------------------------------------------- */
    int v_in_mv = read_channel_mv(V_ADC_CHANNEL);
    int i_in_mv = read_channel_mv(I_ADC_CHANNEL);

        /* --- convert ---------------------------------------------------- */
    g_voltage = ((float)v_in_mv / 1000.0f) * V_DIVIDER;              /* V */
    g_current = ((float)i_in_mv / 1000.0f) / (I_SHUNT_MV_PER_A/1000);/* A */

        /* --- coulomb counter ------------------------------------------- */
        float dt = (float)pdTICKS_TO_MS(T) / 1000.0f;             /* s */
        coulomb_cnt -= g_current * dt;                            /* C */
        float cap_C  = BAT_CAPACITY_AH * 3600.0f;
        g_soc = fmaxf(0.0f, fminf(100.0f, (coulomb_cnt / cap_C) * 100.0f));

        vTaskDelay(T);
    }
}

/* getters (thread-safe: single writer, 32-bit) */
float battery_get_voltage(void){ return g_voltage; }
float battery_get_current(void){ return g_current; }
float battery_get_soc(void)    { return g_soc; }