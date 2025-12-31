#include "battery.hpp"

#include "esp_check.h"
#include "esp_log.h"

static const char *BATTERY_TAG = "BatteryMonitor";

BatteryMonitor::BatteryMonitor(adc_unit_t unit, adc_channel_t channel, adc_atten_t attenuation, float divider_ratio)
    : unit_(unit),
      channel_(channel),
      attenuation_(attenuation),
      divider_ratio_(divider_ratio),
      adc_handle_(nullptr),
      cali_handle_(nullptr),
      initialized_(false)
{
}

esp_err_t BatteryMonitor::init()
{
    if (initialized_) {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = unit_,
        .clk_src = static_cast<adc_oneshot_clk_src_t>(0), // default clock source
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_config, &adc_handle_), BATTERY_TAG, "ADC oneshot init failed");

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = attenuation_,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_handle_, channel_, &chan_config), BATTERY_TAG, "ADC channel config failed");

    setup_calibration(); // best effort; will log if unavailable
    initialized_ = true;
    return ESP_OK;
}

esp_err_t BatteryMonitor::setup_calibration()
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit_,
        .atten = attenuation_,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle_) == ESP_OK) {
        ESP_LOGI(BATTERY_TAG, "Battery ADC calibration: curve fitting");
        return ESP_OK;
    }
    ESP_LOGW(BATTERY_TAG, "Battery ADC calibration (curve fitting) not available");
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit_,
        .atten = attenuation_,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle_) == ESP_OK) {
        ESP_LOGI(BATTERY_TAG, "Battery ADC calibration: line fitting");
        return ESP_OK;
    }
    ESP_LOGW(BATTERY_TAG, "Battery ADC calibration (line fitting) not available");
#else
    ESP_LOGW(BATTERY_TAG, "Battery ADC calibration not supported by this IDF build");
#endif
    return ESP_FAIL;
}

int BatteryMonitor::read_raw()
{
    if (!initialized_) {
        ESP_LOGW(BATTERY_TAG, "Battery ADC not initialized");
        return -1;
    }

    int raw = 0;
    esp_err_t err = adc_oneshot_read(adc_handle_, channel_, &raw);
    if (err != ESP_OK) {
        ESP_LOGE(BATTERY_TAG, "Battery ADC read failed: %s", esp_err_to_name(err));
        return -1;
    }

    return raw;
}

int BatteryMonitor::read_voltage_mv()
{
    int raw = read_raw();
    if (raw < 0) {
        return -1;
    }

    int pin_mv = raw;
    if (cali_handle_) {
        if (adc_cali_raw_to_voltage(cali_handle_, raw, &pin_mv) != ESP_OK) {
            ESP_LOGW(BATTERY_TAG, "Battery ADC calibration conversion failed, using raw");
            pin_mv = raw;
        }
    }

    float battery_mv = static_cast<float>(pin_mv) * divider_ratio_;
    return static_cast<int>(battery_mv + 0.5f); // rounded to nearest mV
}
