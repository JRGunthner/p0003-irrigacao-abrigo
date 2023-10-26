#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sens_tpu.h"
#include "driver/i2c.h"
#include "bme280.h"

#define TAG_BME280  "BME280"

#define I2C_MASTER_ACK  0
#define I2C_MASTER_NACK 1

// Initialize I2C communication parameters
void i2c_master_init(void) {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

// BME280 I2C write function
static int8_t BME280_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
    int32_t iError = BME280_INIT_VALUE;

    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, cnt, true);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
        iError = SUCCESS;
    } else {
        iError = FAIL;
    }
    i2c_cmd_link_delete(cmd);

    return (int8_t)iError;
}

// BME280 I2C read function
static int8_t BME280_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
    int32_t iError = BME280_INIT_VALUE;
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (cnt > 1) {
        i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
        iError = SUCCESS;
    } else {
        iError = FAIL;
    }

    i2c_cmd_link_delete(cmd);

    return (int8_t)iError;
}

// BME280 I2C delay function
void BME280_delay_msek(uint32_t msek) {
    vTaskDelay(msek / portTICK_PERIOD_MS);
}

char temperature[12];
char humidity[10];
char pressure[10];

// BME280 I2C task
void vBme280Task(void *params) {
    // BME280 I2C communication structure
    struct bme280_t bme280 = {
        .bus_write = BME280_I2C_bus_write,
        .bus_read = BME280_I2C_bus_read,
        .dev_addr = BME280_I2C_ADDRESS1,
        .delay_msec = BME280_delay_msek
    };

    int32_t com_rslt;
    int32_t v_uncomp_pressure_s32;
    int32_t v_uncomp_temperature_s32;
    int32_t v_uncomp_humidity_s32;

    // Initialize BME280 sensor and set internal parameters
    com_rslt = bme280_init(&bme280);
    printf("com_rslt %d\n", com_rslt);

    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
    com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
    com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

    com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
    com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

    com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
    if (com_rslt == SUCCESS) {
        while (true) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            // Read BME280 data
            com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

            double temp = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
            //char temperature[12];
            sprintf(temperature, "%.2f°C", temp);

            double press = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100; // Pa -> hPa
            //char pressure[10];
            sprintf(pressure, "%.2f hPa", press);

            double hum = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
            //char humidity[10];
            sprintf(humidity, "%.2f %%", hum);

            // Print BME data
            if (com_rslt == SUCCESS) {
                // printf("Temperatura %s\n",temperature);
                // printf("Pressão atm %s\n",pressure);
                // printf("Umidade atm %s\n",humidity);
            } else {
                ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
            }
        }
    } else {
        ESP_LOGE(TAG_BME280, "init or setting error. code: %d", com_rslt);
    }

    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
