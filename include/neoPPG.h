#ifndef __NEOPPG_H__
#define __NEOPPG_H__

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO           7                // SCL pin for ESP32-C6-Mini Dev Board
#define I2C_MASTER_SDA_IO           6                // SDA pin for ESP32-C6-Mini Dev Board
#define I2C_MASTER_NUM              0                // I2C master number
#define I2C_MASTER_FREQ_HZ          400000          // I2C master clock frequency
#define I2C_MASTER_TIMEOUT_MS       1000            // I2C master timeout

#define DEVICE_ADDRESS              0x57            // I2C device address
#define MAX_SIZE                    40

// Register Addresses
#define REG_DEVICE_ID              0x04
#define REG_SPO2_DATA              0x0C
#define REG_TEMPERATURE            0x14
#define REG_BAUD_RATE             0x06
#define REG_SENSOR_CONTROL        0x20

typedef struct {
    int SPO2;
    int BPM;
} bpm_spo2_t;

typedef struct {
    i2c_port_t i2c_port;
    uint8_t device_addr;
    bpm_spo2_t readings;
} neoPPG_handle_t;

// Function declarations
esp_err_t neoPPG_init(neoPPG_handle_t *dev);
esp_err_t neoPPG_measure(neoPPG_handle_t *dev);
esp_err_t neoPPG_get_temperature(neoPPG_handle_t *dev, float *temperature);
esp_err_t neoPPG_start_collect(neoPPG_handle_t *dev);
esp_err_t neoPPG_end_collect(neoPPG_handle_t *dev);

#endif