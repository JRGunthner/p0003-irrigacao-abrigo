#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sntp.h"
#include "esp_netif.h"
#include "esp_err.h"

#include "modbus_params.h"
#include "mbcontroller.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "bme280.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#define TAG_BME280 "BME280"

#define SDA_PIN GPIO_NUM_1
#define SCL_PIN GPIO_NUM_2

#define I2C_MASTER_ACK  0
#define I2C_MASTER_NACK 1

#define RELE_1 GPIO_NUM_8
#define RELE_2 GPIO_NUM_19
#define RELE_3 GPIO_NUM_20
#define RELE_4 GPIO_NUM_3

#define BOTAO_ESC GPIO_NUM_14
#define BOTAO_ENT GPIO_NUM_13

#define UART_PIN_TX  GPIO_NUM_11
#define UART_PIN_RX  GPIO_NUM_10
#define UART_PIN_RTS GPIO_NUM_9

#define LED_BR GPIO_NUM_5

#define MB_PORT_NUM  2
#define MB_BAUD_RATE 19200

// The number of parameters that intended to be used in the particular control process
#define MASTER_MAX_CIDS num_device_parameters

// Timeout to update cid over Modbus
#define UPDATE_CIDS_TIMEOUT_MS   500
#define UPDATE_CIDS_TIMEOUT_TICS UPDATE_CIDS_TIMEOUT_MS / portTICK_RATE_MS

// Timeout between polls
#define POLL_TIMEOUT_MS   1
#define POLL_TIMEOUT_TICS POLL_TIMEOUT_MS / portTICK_RATE_MS

typedef struct {
    uint16_t test_regs_1[150];
    uint16_t test_regs_2[150];
    uint16_t test_regs_3[150];
} holding_reg_params_tche;
#define HOLD_OFFSET(field)  ((uint16_t)(offsetof(holding_reg_params_tche, field) + 1))

#define STR(fieldname) ((const char*)( fieldname ))
// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

// Endereço do inversor
#define ENDERECO_INVERSOR   22

// Parâmetros do inversor
#define PARAM_ESTADO_LOGICO 680
#define PARAM_VER_VELOC     681
#define PARAM_PALAVRA_CTRL  682
#define PARAM_REF_VELOC     683

// Funções disponíveis no inversor
#define FUNC_READ_HOLDING_REG   3
#define FUNC_WRITE_SINGLE_REG   6
#define FUNC_WRITE_MULT_REG     16
#define FUNC_READ_DEVICE_INFO   16

static const char *TAG = "INVERSOR_TRI";

// CIDs dos parâmetros
enum {
    CID_PARAM_VELOCIDADE = 0,
    CID_DESLIGA_MOTOR,
    CID_LIGA_MOTOR
};

// Example Data (Object) Dictionary for Modbus parameters:
// The CID              field in the table must be unique.
// Modbus Slave Addr    field defines slave address of the device with correspond parameter.
// Modbus Reg Type      Type of Modbus register area (Holding register, Input Register and such).
// Reg Start            field defines the start Modbus register number and Reg Size defines the number of registers for the characteristic accordingly.
// The Instance Offset  defines offset in the appropriate parameter structure that will be used as instance to save parameter value.
// Data Type            Data Size specify type of the characteristic and its data size.
// Parameter Options    field specifies the options that can be used to process parameter value (limits or masks).
// Access Mode          can be used to implement custom options for processing of characteristic (Read/Write restrictions, factory mode values and etc).
const mb_parameter_descriptor_t device_parameters[] = {
    {
        CID_PARAM_VELOCIDADE,       // CID,
        STR("Velocidade do motor"), // Param Name,
        STR(""),                    // Units,
        ENDERECO_INVERSOR,          // Modbus Slave Addr,
        MB_PARAM_HOLDING,           // Modbus Reg Type,
        PARAM_REF_VELOC,            // Reg Start,
        1,                          // Reg Size,
        HOLD_OFFSET(test_regs_1),   // Instance Offset,
        PARAM_TYPE_U16,             // Data Type,
        2,                          // Data Size,
        OPTS(0, 0x2000, 1),         // Parameter Options,
        FUNC_WRITE_SINGLE_REG       // Access Mode
    }, {
        CID_DESLIGA_MOTOR,          // CID,
        STR("Desliga motor"),       // Param Name,
        STR(""),                    // Units,
        ENDERECO_INVERSOR,          // Modbus Slave Addr,
        MB_PARAM_HOLDING,           // Modbus Reg Type,
        PARAM_PALAVRA_CTRL,         // Reg Start,
        2,                          // Reg Size,
        HOLD_OFFSET(test_regs_2),     // Instance Offset,
        PARAM_TYPE_U16,             // Data Type,
        2,                          // Data Size,
        OPTS(0, 0x2000, 1),         // Parameter Options,
        FUNC_WRITE_SINGLE_REG       // Access Mode
    }, {
        CID_LIGA_MOTOR,             // CID,
        STR("Liga motor"),          // Param Name,
        STR(""),                    // Units,
        ENDERECO_INVERSOR,          // Modbus Slave Addr,
        MB_PARAM_HOLDING,           // Modbus Reg Type,
        PARAM_PALAVRA_CTRL,         // Reg Start,
        2,                          // Reg Size,
        HOLD_OFFSET(test_regs_3),   // Instance Offset,
        PARAM_TYPE_U16,             // Data Type,
        2,                          // Data Size,
        OPTS(0, 0x2000, 1),         // Parameter Options,
        FUNC_WRITE_SINGLE_REG       // Access Mode
    },
};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));

// The function to get pointer to parameter storage (instance) according to parameter description table
static void* master_get_param_data(const mb_parameter_descriptor_t* param_descriptor) {
    assert(param_descriptor != NULL);
    void* instance_ptr = NULL;
    if (param_descriptor->param_offset != 0) {
        instance_ptr = ((void*)&holding_reg_params + param_descriptor->param_offset - 1);
    } else {
        ESP_LOGE(TAG, "Wrong parameter offset for CID #%d", param_descriptor->cid);
        assert(instance_ptr != NULL);
    }
    return instance_ptr;
}

// Modbus master initialization
static esp_err_t master_init(void) {
    esp_err_t err;
    mb_communication_info_t comm = {
        .port = MB_PORT_NUM,
        .slave_addr = 22,
        .mode = MB_MODE_RTU,
        .baudrate = MB_BAUD_RATE,
        .parity = MB_PARITY_NONE
    };
    void* master_handler = NULL;

    err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MB_RETURN_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, TAG, "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb controller initialization fail, returns(0x%x).", (uint32_t)err);

    err = mbc_master_setup((void*)&comm);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb controller setup fail, returns(0x%x).", (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, UART_PIN_TX, UART_PIN_RX, UART_PIN_RTS, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);

    err = mbc_master_start();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb controller start fail, returns(0x%x).", (uint32_t)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb controller set descriptor fail, returns(0x%x).", (uint32_t)err);
    printf("Modbus inicialidado\r\n");
    return err;
}

static void motor_definir_velocidade(uint16_t rpm) {
    esp_err_t err = ESP_OK;
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    uint16_t velocidade = (0x2000 * rpm) / 3470;

    for (uint16_t cid = 0; (err != ESP_ERR_NOT_FOUND) && cid < MASTER_MAX_CIDS; cid++) {
        err = mbc_master_get_cid_info(cid, &param_descriptor);

        if ((err == ESP_ERR_NOT_FOUND) && (param_descriptor == NULL))
            break;

        void* temp_data_ptr = master_get_param_data(param_descriptor);
        assert(temp_data_ptr);
        uint8_t type = 0;

        if (param_descriptor->cid == CID_PARAM_VELOCIDADE) {
            err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key, (uint8_t*)temp_data_ptr, &type);
            if (err == ESP_OK) {
                printf("Parâmetro %d - %s. Valor: 0x%08x. Leitura.\r\n", param_descriptor->cid, (char*)param_descriptor->param_key, *(uint32_t*)temp_data_ptr);

                memset((void*)temp_data_ptr, velocidade, param_descriptor->param_size);
                *(uint32_t*)temp_data_ptr = velocidade;
                err = mbc_master_set_parameter(cid, (char*)param_descriptor->param_key, (uint8_t*)temp_data_ptr, &type);
                if (err == ESP_OK) {
                    printf("Parâmetro %d - %s. Valor: 0x%08x. Escrita.\r\n", param_descriptor->cid, (char*)param_descriptor->param_key, *(uint32_t*)temp_data_ptr);
                } else {
                    ESP_LOGE(TAG, "Parâmetro %d - %s. Falha na escrita. ERRO 0x%x (%s).", param_descriptor->cid, (char*)param_descriptor->param_key, (int)err, (char*)esp_err_to_name(err));
                }
            } else {
                ESP_LOGE(TAG, "Parâmetro %d - %s. Falha na leitura. ERRO 0x%x (%s).", param_descriptor->cid, (char*)param_descriptor->param_key, (int)err, (char*)esp_err_to_name(err));
            }
        }
        vTaskDelay(POLL_TIMEOUT_TICS);
    }
    vTaskDelay(UPDATE_CIDS_TIMEOUT_TICS);
}

static void motor_desliga(void) {
    esp_err_t err = ESP_OK;
    const mb_parameter_descriptor_t* param_descriptor = NULL;

    for (uint16_t cid = 0; (err != ESP_ERR_NOT_FOUND) && cid < MASTER_MAX_CIDS; cid++) {
        err = mbc_master_get_cid_info(cid, &param_descriptor);

        if ((err == ESP_ERR_NOT_FOUND) && (param_descriptor == NULL))
            break;

        void* temp_data_ptr = master_get_param_data(param_descriptor);
        assert(temp_data_ptr);
        uint8_t type = 0;

        if (param_descriptor->cid == CID_DESLIGA_MOTOR) {
            err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key, (uint8_t*)temp_data_ptr, &type);
            if (err == ESP_OK) {
                printf("Parâmetro %d - %s. Valor: 0x%08x. Leitura.\r\n", param_descriptor->cid, (char*)param_descriptor->param_key, *(uint32_t*)temp_data_ptr);
                memset((void*)temp_data_ptr, 0x12, param_descriptor->param_size);
                *(uint32_t*)temp_data_ptr =  0x12;
                err = mbc_master_set_parameter(cid, (char*)param_descriptor->param_key, (uint8_t*)temp_data_ptr, &type);
                if (err == ESP_OK) {
                    printf("Parâmetro %d - %s. Valor: 0x%08x. Escrita.\r\n", param_descriptor->cid, (char*)param_descriptor->param_key, *(uint32_t*)temp_data_ptr);
                } else {
                    ESP_LOGE(TAG, "Parâmetro %d - %s. Falha na escrita. ERRO 0x%x (%s).", param_descriptor->cid, (char*)param_descriptor->param_key, (int)err, (char*)esp_err_to_name(err));
                }
            } else {
                ESP_LOGE(TAG, "Parâmetro %d - %s. Falha na leitura. ERRO 0x%x (%s).", param_descriptor->cid, (char*)param_descriptor->param_key, (int)err, (char*)esp_err_to_name(err));
            }
        }
        vTaskDelay(POLL_TIMEOUT_TICS);
    }
    vTaskDelay(UPDATE_CIDS_TIMEOUT_TICS);
}

static void motor_liga(void) {
    esp_err_t err = ESP_OK;
    const mb_parameter_descriptor_t* param_descriptor = NULL;

    for (uint16_t cid = 0; (err != ESP_ERR_NOT_FOUND) && cid < MASTER_MAX_CIDS; cid++) {
        err = mbc_master_get_cid_info(cid, &param_descriptor);

        if ((err == ESP_ERR_NOT_FOUND) && (param_descriptor == NULL))
            break;

        void* temp_data_ptr = master_get_param_data(param_descriptor);
        assert(temp_data_ptr);
        uint8_t type = 0;

        if (param_descriptor->cid == CID_LIGA_MOTOR) {
            err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key, (uint8_t*)temp_data_ptr, &type);
            if (err == ESP_OK) {
                printf("Parâmetro %d - %s. Valor: 0x%08x. Leitura.\r\n", param_descriptor->cid, (char*)param_descriptor->param_key, *(uint32_t*)temp_data_ptr);
                memset((void*)temp_data_ptr, 0x1b, param_descriptor->param_size);
                *(uint32_t*)temp_data_ptr =  0x1b;
                err = mbc_master_set_parameter(cid, (char*)param_descriptor->param_key, (uint8_t*)temp_data_ptr, &type);
                if (err == ESP_OK) {
                    printf("Parâmetro %d - %s. Valor: 0x%08x. Escrita.\r\n", param_descriptor->cid, (char*)param_descriptor->param_key, *(uint32_t*)temp_data_ptr);
                } else {
                    ESP_LOGE(TAG, "Parâmetro %d - %s. Falha na escrita. ERRO 0x%x (%s).", param_descriptor->cid, (char*)param_descriptor->param_key, (int)err, (char*)esp_err_to_name(err));
                }
            } else {
                ESP_LOGE(TAG, "Parâmetro %d - %s. Falha na leitura. ERRO 0x%x (%s).", param_descriptor->cid, (char*)param_descriptor->param_key, (int)err, (char*)esp_err_to_name(err));
            }
        }
        vTaskDelay(POLL_TIMEOUT_TICS);
    }
    vTaskDelay(UPDATE_CIDS_TIMEOUT_TICS);
}

void rele_init(void) {
    gpio_reset_pin(RELE_1);
    gpio_set_direction(RELE_1, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_1, 1);

    gpio_reset_pin(RELE_2);
    gpio_set_direction(RELE_2, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_2, 1);

    gpio_reset_pin(RELE_3);
    gpio_set_direction(RELE_3, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_3, 1);

    gpio_reset_pin(RELE_4);
    gpio_set_direction(RELE_4, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_4, 1);

    
    gpio_reset_pin(LED_BR);
    gpio_set_direction(LED_BR, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_BR, 0);
}

void botao_init(void) {
    gpio_reset_pin(BOTAO_ESC);
    gpio_set_direction(BOTAO_ESC, GPIO_MODE_INPUT);

    gpio_reset_pin(BOTAO_ENT);
    gpio_set_direction(BOTAO_ENT, GPIO_MODE_INPUT);
}

void rele_start_stop(uint8_t rele, bool estado) {
    gpio_set_level(rele, estado);
}

#define RELE_DESACIONA_LIGA rele_start_stop(RELE_1, 0)
#define RELE_DESACIONA_DESL rele_start_stop(RELE_1, 1)
#define RELE_ACIONA_LIGA rele_start_stop(RELE_2, 0)
#define RELE_ACIONA_DESL rele_start_stop(RELE_2, 1)
#define RELE_SAIDA_INVERSOR_LIGA rele_start_stop(RELE_3, 0)
#define RELE_SAIDA_INVERSOR_DESL rele_start_stop(RELE_3, 1)
#define RELE_SELECAO_MANUAL   rele_start_stop(RELE_4, 0)
#define RELE_SELECAO_INVERSOR rele_start_stop(RELE_4, 1)

// Rotina WiFi //////////////////////////////////////////////////
static void obtain_time(void);
static void initialize_sntp(void);

void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

struct tm data;

void rotina_sntp(void) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_year < (2023 - 1900)) {
        printf("Time is not set yet. Connecting to WiFi and getting time over NTP.\r\n");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }

    // Set timezone to Brazil Standard Time
    setenv("TZ", "BRST+3BRDT+2,M10.3.0,M2.3.0", 1);
    tzset();
    localtime_r(&now, &timeinfo);

    while (1) {
        time_t tt = time(NULL);
        data = *gmtime(&tt);
        printf("%02d:%02d:%02d\r\n", data.tm_hour - 3, data.tm_min, data.tm_sec);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
////////////////////////////////////////////////////////////

void delay_s(uint16_t segundos) {
    vTaskDelay((segundos * 1000) / portTICK_PERIOD_MS);
}

void delay_ms(uint16_t milisegundos) {
    vTaskDelay(milisegundos / portTICK_PERIOD_MS);
}

void aciona_aspersor (uint16_t tempo) {
    RELE_SAIDA_INVERSOR_LIGA;
    vTaskDelay(300 / portTICK_PERIOD_MS);
    motor_liga();
    motor_definir_velocidade(3470);

    for (uint16_t i = 0; i < (tempo * 10); i++) {
        if (!gpio_get_level(BOTAO_ESC))
            break;
        delay_ms(100);
    }

    motor_desliga();
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    RELE_SAIDA_INVERSOR_DESL;
}

void ligar_no_horario(uint8_t hh, uint8_t mm, uint8_t ss, uint16_t tempo) {
    if ((data.tm_hour - 3) == hh)
        if (data.tm_min == mm)
            if (data.tm_sec == ss)
                aciona_aspersor(tempo);
}

/////////// BME280 /////////////////////////////////////////////

// Initialize I2C communication parameters
void i2c_master_init() {
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
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
	s32 iError = BME280_INIT_VALUE;

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

	return (s8)iError;
}

// BME280 I2C read function
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
	s32 iError = BME280_INIT_VALUE;
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

	return (s8)iError;
}

// BME280 I2C delay function
void BME280_delay_msek(u32 msek) {
	vTaskDelay(msek / portTICK_PERIOD_MS);
}

char temperature[12];
char humidity[10];
char pressure[10];

// BME280 I2C task
void Publisher_Task(void *params) {
	// BME280 I2C communication structure
	struct bme280_t bme280 = {
		.bus_write = BME280_I2C_bus_write,
		.bus_read = BME280_I2C_bus_read,
		.dev_addr = BME280_I2C_ADDRESS1,
		.delay_msec = BME280_delay_msek
	};

	s32 com_rslt;
	s32 v_uncomp_pressure_s32;
	s32 v_uncomp_temperature_s32;
	s32 v_uncomp_humidity_s32;

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
////////////////////////////////////////////////////////////////

void app_main(void) {
    // Initialize memory
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize I2C parameters
    i2c_master_init();

    // Read the data from BME280 sensor
    xTaskCreate(Publisher_Task, "Publisher_Task", 1024 * 5, NULL, 5, NULL);

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_year < (2023 - 1900)) {
        printf("Time is not set yet. Connecting to WiFi and getting time over NTP.\r\n");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }

    // Set timezone to Brazil Standard Time
    setenv("TZ", "BRST+3BRDT+2,M10.3.0,M2.3.0", 1);
    tzset();
    localtime_r(&now, &timeinfo);
        
    ESP_ERROR_CHECK(master_init());
    rele_init();


    printf("Inicialização Tri!\r\n");

    // Inicialização dos relés
    RELE_DESACIONA_DESL;
    RELE_ACIONA_DESL;
    RELE_SAIDA_INVERSOR_DESL;
    RELE_SELECAO_INVERSOR;

    // Seleciona comando manual. Ativa as botoeiras e
    // não permite acionamento pelo inversor
    //RELE_SELECAO_MANUAL;

    uint8_t s_ant = 0;

    while(1) {
        if (!gpio_get_level(BOTAO_ENT)) {
            // RELE_ACIONA_LIGA;
            // RELE_DESACIONA_DESL;
            // vTaskDelay(100 / portTICK_PERIOD_MS);
            // RELE_ACIONA_DESL;

            aciona_aspersor(120);
        } else if (!gpio_get_level(BOTAO_ESC)) {
            // RELE_DESACIONA_LIGA;
            // RELE_ACIONA_DESL;
            // vTaskDelay(100 / portTICK_PERIOD_MS);
            // RELE_DESACIONA_DESL;

            motor_desliga();
            vTaskDelay(6000 / portTICK_PERIOD_MS);
            RELE_SAIDA_INVERSOR_DESL;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);

        time_t tt = time(NULL);
        data = *gmtime(&tt);

        if (data.tm_sec != s_ant) {
            if ((data.tm_hour - 3) >= 0)
                gpio_set_level(LED_BR, 1);
            else
                gpio_set_level(LED_BR, 0);
            printf("%02d:%02d:%02d, Temperatura: %s, Pressão: %s, Umidade: %s\r\n", data.tm_hour - 3, data.tm_min, data.tm_sec, temperature, pressure, humidity);
            s_ant = data.tm_sec;
        }

        uint16_t tempo_irrigacao = 150; // Em segundos

        ligar_no_horario( 7, 0, 0, tempo_irrigacao);
        ligar_no_horario( 8, 0, 0, tempo_irrigacao);
        ligar_no_horario( 9, 0, 0, tempo_irrigacao);
        ligar_no_horario(10, 0, 0, tempo_irrigacao);
        ligar_no_horario(11, 0, 0, tempo_irrigacao);
        ligar_no_horario(12, 0, 0, tempo_irrigacao);
        ligar_no_horario(13, 0, 0, tempo_irrigacao);
        ligar_no_horario(14, 0, 0, tempo_irrigacao);
        ligar_no_horario(15, 0, 0, tempo_irrigacao);
        ligar_no_horario(16, 0, 0, tempo_irrigacao);
        ligar_no_horario(17, 0, 0, tempo_irrigacao);
        ligar_no_horario(18, 0, 0, tempo_irrigacao);
    }
}

// Rotina que pega a hora oficial de Brasília ///////////////////////
static void obtain_time(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // NTP server address could be aquired via DHCP,
    // see LWIP_DHCP_GET_NTP_SRV menuconfig option
#ifdef LWIP_DHCP_GET_NTP_SRV
    sntp_servermode_dhcp(1);
#endif

    // This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    // Read "Establishing Wi-Fi or Ethernet Connection" section in
    // examples/protocols/README.md for more information about this function.
    ESP_ERROR_CHECK(example_connect());

    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 20;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Esperando pela hora do servidor... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);

    ESP_ERROR_CHECK(example_disconnect());
}

static void initialize_sntp(void) {
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "a.st1.ntp.br");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
}
