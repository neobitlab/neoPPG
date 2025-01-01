#include <string.h>
#include "neoPPG.h"

static const char *TAG = "neoPPG";

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_read_reg(neoPPG_handle_t *dev, uint16_t reg_addr, uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    // Write Register Address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
    // Read Data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->device_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(dev->i2c_port, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_write_reg(neoPPG_handle_t *dev, uint16_t reg_addr, uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(dev->i2c_port, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t neoPPG_init(neoPPG_handle_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C Master Init failed!");
        return ret;
    }

    dev->i2c_port = I2C_MASTER_NUM;
    dev->device_addr = DEVICE_ADDRESS;

    // Check Device ID
    uint8_t id_buf[2];
    ret = i2c_read_reg(dev, REG_DEVICE_ID, id_buf, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Device ID!");
        return ret;
    }

    uint16_t device_id = (id_buf[0] << 8) | id_buf[1];
    if (device_id != 0x0020) {
        ESP_LOGE(TAG, "Invalid Device ID: 0x%04x", device_id);
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

esp_err_t neoPPG_measure(neoPPG_handle_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[8];
    esp_err_t ret = i2c_read_reg(dev, REG_SPO2_DATA, data, 8);
    if (ret != ESP_OK) {
        return ret;
    }

    dev->readings.SPO2 = data[0];
    if (dev->readings.SPO2 == 0) {
        dev->readings.SPO2 = -1;
    }

    dev->readings.BPM = ((uint32_t)data[2] << 24) | 
                            ((uint32_t)data[3] << 16) | 
                            ((uint32_t)data[4] << 8) | 
                            ((uint32_t)data[5]);
    if (dev->readings.BPM == 0) {
        dev->readings.BPM = -1;
    }

    return ESP_OK;
}

esp_err_t neoPPG_get_temperature(neoPPG_handle_t *dev, float *temperature)
{
    if (dev == NULL || temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t temp_buf[2];
    esp_err_t ret = i2c_read_reg(dev, REG_TEMPERATURE, temp_buf, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    *temperature = temp_buf[0] + (temp_buf[1] / 100.0f);
    return ESP_OK;
}

esp_err_t neoPPG_start_collect(neoPPG_handle_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t wbuf[2] = {0, 1};
    return i2c_write_reg(dev, REG_SENSOR_CONTROL, wbuf, 2);
}

esp_err_t neoPPG_end_collect(neoPPG_handle_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t wbuf[2] = {0, 2};
    return i2c_write_reg(dev, REG_SENSOR_CONTROL, wbuf, 2);
}