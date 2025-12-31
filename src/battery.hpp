#ifndef BATTERY_HPP
#define BATTERY_HPP

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"

class BatteryMonitor {
public:
    BatteryMonitor(adc_unit_t unit, adc_channel_t channel, adc_atten_t attenuation, float divider_ratio = 1.0f);

    esp_err_t init();
    int read_raw();
    int read_voltage_mv(); // scaled back to battery voltage using divider ratio

    void set_divider_ratio(float ratio) { divider_ratio_ = ratio; }
    float divider_ratio() const { return divider_ratio_; }

private:
    esp_err_t setup_calibration();

    adc_unit_t unit_;
    adc_channel_t channel_;
    adc_atten_t attenuation_;
    float divider_ratio_;

    adc_oneshot_unit_handle_t adc_handle_;
    adc_cali_handle_t cali_handle_;
    bool initialized_;
};

#endif // BATTERY_HPP
