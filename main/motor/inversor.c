#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "inversor.h"
#include "modbus_params.h"
#include "mbcontroller.h"
#include "esp_log.h"

#define MB_PORT_NUM  2
#define MB_BAUD_RATE 19200

motor_t motor = {
    .estado = DESLIGADO,
    .rpm = 3470,
    .tempo = 0
};

#define MB_RET_ON_FALSE(a, err_code, tag, format, ...) do {                              \
    if (!(a)) {                                                                             \
        ESP_LOGE(tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__); \
        return err_code;                                                                    \
    }                                                                                       \
} while(0)

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
        HOLD_OFFSET(test_regs_2),   // Instance Offset,
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
esp_err_t inversor_init(void) {
    esp_err_t err;
    mb_communication_info_t comm = {
        .port = MB_PORT_NUM,
        .slave_addr = 22,
        .mode = MB_MODE_RTU,
        .baudrate = MB_BAUD_RATE,
        .parity = MB_PARITY_NONE
    };
    void* master_handler = NULL;

    motor.estado = DESLIGADO;
    
    semaph_motor_ligar = xSemaphoreCreateBinary();
    semaph_motor_desligar = xSemaphoreCreateBinary();

    err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MB_RET_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, TAG, "mb controller initialization fail.");
    MB_RET_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb controller initialization fail, returns(0x%x).", (uint32_t)err);

    err = mbc_master_setup((void*)&comm);
    MB_RET_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb controller setup fail, returns(0x%x).", (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, UART_PIN_TX, UART_PIN_RX, UART_PIN_RTS, UART_PIN_NO_CHANGE);
    MB_RET_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);

    err = mbc_master_start();
    MB_RET_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb controller start fail, returns(0x%x).", (uint32_t)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RET_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MB_RET_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG, "mb controller set descriptor fail, returns(0x%x).", (uint32_t)err);
    printf("Modbus inicialidado\r\n");
    return err;
}

void inversor_velocidade_motor(uint16_t rpm) {
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

void inversor_desligar_motor(void) {
    esp_err_t err = ESP_OK;
    const mb_parameter_descriptor_t* param_descriptor = NULL;

    motor.estado = DESLIGANDO;

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

    xSemaphoreGive(semaph_motor_desligar);
    motor.estado = DESLIGADO;
}

void inversor_ligar_motor(void) {
    esp_err_t err = ESP_OK;
    const mb_parameter_descriptor_t* param_descriptor = NULL;

    motor.estado = LIGANDO;

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

    xSemaphoreGive(semaph_motor_ligar);
    motor.estado = LIGADO;
}
