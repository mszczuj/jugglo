#ifndef IMU_HPP
#define IMU_HPP

#include <esp_timer.h>
#include <freertos/mpu_wrappers.h>
#include <freertos/projdefs.h>
#include <freertos/task.h>
#include <esp_log.h>


#define IMUBASETAG "IMUBase"



class IMUBase {
public:
	virtual ~IMUBase() {}

    virtual int initialize() = 0;

    virtual int update() = 0;

    virtual void getAccel(int16_t* out) = 0;
	virtual void getGyro(int16_t* out) = 0;

    void waitAndUpdate() {

        if (last_read_time == 0) {
            // seed timestamp on first use to avoid a bogus overrun and keep cadence
            last_read_time = esp_timer_get_time();
        }
        
        uint64_t time_since_last_read = esp_timer_get_time() - last_read_time;
        
        if (time_since_last_read > expected_interval_us) {
            ESP_LOGW(IMUBASETAG, "IMU read overrun! (main loop working too slow? %d)", time_since_last_read);
        } else {
            // wait to maintain expected update rate of the IMU (TODO use a more precise method?)
            int64_t delay_us = static_cast<int64_t>(expected_interval_us) - static_cast<int64_t>(time_since_last_read);
            if (delay_us > 0) {
                esp_rom_delay_us(delay_us);
            }
        }    

        // important to measure before the update() call, which takes some time
        last_read_time = esp_timer_get_time() ;
        update();    
        
    }

protected:
    float aRes; // accelerometer resolution
    float gRes; // gyroscope resolution

    uint16_t expected_interval_us = 0;

private: 
    uint64_t last_read_time = 0;
    
};


#endif // IMU_HPP
