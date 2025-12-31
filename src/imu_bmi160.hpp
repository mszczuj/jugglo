#ifndef IMU_BMI160_HPP
#define IMU_BMI160_HPP

#include "imu_base.hpp"
#include <cstdint>
#include <cassert>
#include <cstring>
#include <freertos/projdefs.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>

#define IMUTAG "IMUBMI160"

/*

	BMI160 REGISTERS

*/
#define BMI160_CMD 0x7E
#define BMI160_STEP_CNT_1 0x79
#define BMI160_STEP_CNT_0 0x78
#define BMI160_OFFSET_6 0x77
#define BMI160_OFFSET_5 0x76
#define BMI160_OFFSET_4 0x75
#define BMI160_OFFSET_3 0x74
#define BMI160_OFFSET_2 0x73
#define BMI160_OFFSET_1 0x72
#define BMI160_OFFSET_0 0x71
#define BMI160_NV_CONF 0x70
#define BMI160_SELF_TEST 0x6D
#define BMI160_PMU_TRIGGER 0x6C
#define BMI160_IF_CONF 0x6B
#define BMI160_CONF 0x6A
#define BMI160_FOC_CONF 0x69
#define BMI160_INT_FLAT_1 0x68
#define BMI160_INT_FLAT_0 0x67
#define BMI160_INT_ORIENT_1 0x66
#define BMI160_INT_ORIENT_0 0x65
#define BMI160_INT_TAP_1 0x64
#define BMI160_INT_TAP_0 0x63
#define BMI160_INT_MOTION_3 0x62
#define BMI160_INT_MOTION_2 0x61
#define BMI160_INT_MOTION_1 0x60
#define BMI160_INT_MOTION_0 0x5F
#define BMI160_INT_LOWHIGH_4 0x5E
#define BMI160_INT_LOWHIGH_3 0x5D
#define BMI160_INT_LOWHIGH_2 0x5C
#define BMI160_INT_LOWHIGH_1 0x5B
#define BMI160_INT_LOWHIGH_0 0x5A
#define BMI160_INT_DATA_1 0x59
#define BMI160_INT_DATA_0 0x58
#define BMI160_INT_MAP_2 0x57
#define BMI160_INT_MAP_1 0x56
#define BMI160_INT_MAP_0 0x55
#define BMI160_INT_LATCH 0x54
#define BMI160_INT_OUT_CTRL 0x53
#define BMI160_INT_EN_2 0x52
#define BMI160_INT_EN_1 0x51
#define BMI160_INT_EN_0 0x50
#define BMI160_MAG_IF_4 0x4F
#define BMI160_MAG_IF_3 0x4E
#define BMI160_MAG_IF_2 0x4D
#define BMI160_MAG_IF_1 0x4C
#define BMI160_MAG_IF_0 0x4B
#define BMI160_FIFO_CONFIG_1 0x47
#define BMI160_FIFO_CONFIG_0 0x46
#define BMI160_FIFO_DOWNS 0x45
#define BMI160_MAG_CONF 0x44
#define BMI160_GYR_RANGE 0x43
#define BMI160_GYR_CONF 0x42
#define BMI160_ACC_RANGE 0x41
#define BMI160_ACC_CONF 0x40
#define BMI160_FIFO_DATA 0x24
#define BMI160_FIFO_LENGTH_1 0x23
#define BMI160_FIFO_LENGTH_0 0x22
#define BMI160_TEMPERATURE_1 0x21
#define BMI160_TEMPERATURE_0 0x20
#define BMI160_INT_STATUS_3 0x1F
#define BMI160_INT_STATUS_2 0x1E
#define BMI160_INT_STATUS_1 0x1D
#define BMI160_INT_STATUS_0	0x1C
#define BMI160_STATUS 0x1B
#define BMI160_SENSOR_TIME_H 0x1A
#define BMI160_SENSOR_TIME_M 0x19
#define BMI160_SENSOR_TIME_L 0x18
#define BMI160_ACC_Z_H 0x17
#define BMI160_ACC_Z_L 0x16
#define BMI160_ACC_Y_H 0x15
#define BMI160_ACC_Y_L 0x14
#define BMI160_ACC_X_H 0x13
#define BMI160_ACC_X_L 0x12
#define BMI160_GYR_Z_H 0x11
#define BMI160_GYR_Z_L 0x10
#define BMI160_GYR_Y_H 0x0F
#define BMI160_GYR_Y_L 0x0E
#define BMI160_GYR_X_H 0x0D
#define BMI160_GYR_X_L 0x0C
#define BMI160_RHALL_H 0x0B
#define BMI160_RHALL_L 0x0A
#define BMI160_MAG_Z_H 0x09
#define BMI160_MAG_Z_L 0x08
#define BMI160_MAG_Y_H 0x07
#define BMI160_MAG_Y_L 0x06
#define BMI160_MAG_X_H 0x05
#define BMI160_MAG_X_L 0x04
#define BMI160_PMU_STATUS 0x03
#define BMI160_ERR_REG 0x02
#define BMI160_CHIP_ID 0x00
#define BMI160_CHIP_ID_DEFAULT_VALUE 0xD1
#define BMI160_DEFAULT_ADDRESS 0x69


class IMUBMI160 : public IMUBase {
public:
    IMUBMI160() : dev_handle(nullptr), bus_handle(nullptr), i2c_address(BMI160_DEFAULT_ADDRESS) {

    };
  
    int initialize(i2c_master_bus_handle_t bus, uint8_t address = BMI160_DEFAULT_ADDRESS) {
        bus_handle = bus;
        i2c_address = address;

        // Verify communication with BMI160
        uint8_t chip_id = 0;
        addDeviceIfNeeded();
        readRegister(BMI160_CHIP_ID, &chip_id, 1);

        assert(chip_id == BMI160_CHIP_ID_DEFAULT_VALUE);
        
        writeCommand(0xB6); // reset device
        vTaskDelay(pdMS_TO_TICKS(100));

        writeCommand(0x11); // put accelerometer in normal mode
        vTaskDelay(pdMS_TO_TICKS(100));

        writeCommand(0x15); // put gyro in normal mode
        vTaskDelay(pdMS_TO_TICKS(100));

        writeRegister(BMI160_ACC_RANGE, 0x0C); // Set up full scale Accel range. +-16G
        writeRegister(BMI160_GYR_RANGE, 0x00); // Set up full scale Gyro range. +-2000dps

        writeRegister(BMI160_ACC_CONF, 0x0A); // Set Accel ODR to 400hz, BWP mode to Oversample 4, LPF of ~40.5hz
        writeRegister(BMI160_GYR_CONF, 0x0A); // Set Gyro ODR to 400hz, BWP mode to Oversample 4, LPF of ~34.15hz
        writeRegister(BMI160_FIFO_CONFIG_0, 0x00); // No downsampling, watermark disabled
        writeRegister(BMI160_FIFO_CONFIG_1, 0xC0); // Enable accel + gyro in FIFO, header/time disabled (12-byte frames)

        ESP_LOGI(IMUTAG, "BMI160 initialized (chip_id=%d)", chip_id);

        aRes = 16.f / 32768.f;			//ares value for full range (16g) readings
	    gRes = 2000.f / 32768.f;	    //gres value for full range (2000dps) readings
        expected_interval_us = 2500; // 400Hz update rate

        return ESP_OK;
    };

    int initialize() override {
        // Fallback for compatibility: bus must be set by caller via two-arg initialize
        assert(bus_handle != nullptr);
        return initialize(bus_handle, i2c_address);
    }

    int update() override {

        // Read FIFO byte count (11-bit value across two registers)
        uint8_t fifo_len_raw[2] = {0};
        readRegister(BMI160_FIFO_LENGTH_0, fifo_len_raw, sizeof(fifo_len_raw));
        uint16_t fifo_len = ((uint16_t)fifo_len_raw[1] << 8) | fifo_len_raw[0];
        fifo_len &= 0x07FF; // FIFO length uses 11 bits

        if (fifo_len == 0) 
            return 0;               

        // Limit to our local buffer; if FIFO overran, we drop the oldest tail.
        fifo_len = (fifo_len > sizeof(fifo_buffer)) ? sizeof(fifo_buffer) : fifo_len;
        readRegister(BMI160_FIFO_DATA, fifo_buffer, fifo_len);

        // Parse the FIFO assuming accel + gyro in header-less mode (12 bytes per frame).
        size_t frames = fifo_len / FIFO_FRAME_BYTES;
        size_t offset = fifo_len - frames * FIFO_FRAME_BYTES; // skip any trailing partial bytes

        // Copy the entire FIFO buffer to imu_buffer as int16_t values (little-endian pairs)
        size_t imu_count = fifo_len / 2;
        for (size_t i = 0; i + 1 < fifo_len && i / 2 < sizeof(imu_buffer) / sizeof(int16_t); i += 2) {
            imu_buffer[i / 2] = ((int16_t)fifo_buffer[i + 1] << 8) | fifo_buffer[i];
        }
        
        return fifo_len / FIFO_FRAME_BYTES;
    };

    virtual void getAccel(int16_t* out) override {
        // memcpy(out, &IMUCount[3], 3*sizeof(int16_t));
    };
    
    virtual void getGyro(int16_t* out) override {
        // memcpy(out, &IMUCount[0], 3*sizeof(int16_t));
    };
    int16_t imu_buffer[128];  // local buffer for parsed IMU samples (accel + gyro interleaved)

private:
    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_handle_t bus_handle;
    uint8_t i2c_address;
    
    static constexpr size_t FIFO_FRAME_BYTES = 12;
    uint8_t read_buffer[12];  // x/y/z gyo and accel buffer for i2c reads
    
    // this buffer is to big (we read at max 8 smaples (12 bytes each, so 96 bytes max), but for future proofing
    // if stack problems rise we can reduce it
    uint8_t fifo_buffer[512];   // local scratch for draining BMI160 FIFO

    void addDeviceIfNeeded() {
        if (dev_handle != nullptr) {
            return;
        }
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = i2c_address,
            .scl_speed_hz = 400000,  // 400 kHz is more tolerant when WiFi disables interrupts briefly
            .scl_wait_us = 100,
            .flags = {
                .disable_ack_check = false,
            },
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    }

    void writeCommand(uint8_t command) {
        uint8_t buf[2] = {BMI160_CMD, command};
        ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, sizeof(buf), pdMS_TO_TICKS(1000)));
    };

    void writeRegister(uint8_t reg, uint8_t value) {
        uint8_t buf[2] = {reg, value};
        ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, sizeof(buf), pdMS_TO_TICKS(1000)));
    };

    void readRegister(uint8_t reg, uint8_t* data, size_t len) {
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, pdMS_TO_TICKS(1000)));
    }
        
};  

#endif // IMU_BMI160_HPP
