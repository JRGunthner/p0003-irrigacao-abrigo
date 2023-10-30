#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "bme280.h"

#include "inversor.h"
#include "sens_tpu.h"
#include "teclado.h"
#include "wifi.h"
#include "mqtt.h"
#include "sntp.h"
#include "types.h"

#define TAG "MAIN"
#define DISPOSITIVO_ID 1

xSemaphoreHandle semaph_motor_ligar;
xSemaphoreHandle semaph_motor_desligar;

uint16_t t_irrig = 0;

tipo_acionamento_t tipo_acionamento = INVERSOR;

xTaskHandle xHandle_mqttInitTask = NULL;
xTaskHandle xHandle_mqttRxTask = NULL;
xTaskHandle xHandle_mqttTxTask = NULL;
xTaskHandle xHandle_mainTask = NULL;
xTaskHandle xHandle_sntpTask = NULL;
xTaskHandle xHandle_motorTask = NULL;

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

void rele_start_stop(uint8_t rele, bool estado) {
    gpio_set_level(rele, estado);
}

#define RELE_DESACIONA_LIGA      rele_start_stop(RELE_1, 0)
#define RELE_DESACIONA_DESL      rele_start_stop(RELE_1, 1)
#define RELE_ACIONA_LIGA         rele_start_stop(RELE_2, 0)
#define RELE_ACIONA_DESL         rele_start_stop(RELE_2, 1)
#define RELE_SAIDA_INVERSOR_LIGA rele_start_stop(RELE_3, 0)
#define RELE_SAIDA_INVERSOR_DESL rele_start_stop(RELE_3, 1)
#define RELE_SELECAO_MANUAL      rele_start_stop(RELE_4, 0)
#define RELE_SELECAO_INVERSOR    rele_start_stop(RELE_4, 1)

void delay_s(uint16_t segundos) {
    vTaskDelay((segundos * 1000) / portTICK_PERIOD_MS);
}

void delay_ms(uint16_t milisegundos) {
    vTaskDelay(milisegundos / portTICK_PERIOD_MS);
}

static error_t motor_ligar_inversor(void) {
    RELE_SAIDA_INVERSOR_LIGA;
    delay_ms(300);
    inversor_ligar_motor();
    inversor_velocidade_motor(3470);

    // for (uint16_t i = 0; i < (tempo * 10); i++) {
    //     if (botao_esc()) {
    //         delay_ms(100);
    //         break;
    //     }
    //     delay_ms(100);
    // }

    // delay_ms(100);
    // inversor_desligar_motor();
    // delay_s(6);
    // RELE_SAIDA_INVERSOR_DESL;

    return pdOK;
}

static error_t motor_desligar_inversor(void) {
    inversor_desligar_motor();
    delay_s(6);
    RELE_SAIDA_INVERSOR_DESL;
    return pdOK;
}

static error_t motor_ligar_manual(void) {
    RELE_ACIONA_LIGA;
    RELE_DESACIONA_DESL;
    delay_ms(100);
    RELE_ACIONA_DESL;
    return pdOK;
}

static error_t motor_desligar_manual(void) {
    RELE_DESACIONA_LIGA;
    RELE_ACIONA_DESL;
    delay_ms(100);
    RELE_DESACIONA_DESL;
    return pdOK;
}

static error_t aspersor_ligar(uint16_t tempo) {
    printf("Ligando aspersor\r\n");

    motor.tempo = tempo;

    if (tipo_acionamento == INVERSOR) {
        motor_ligar_inversor();
    } else {
        motor_ligar_manual();
    }
    return pdOK;
}

static error_t aspersor_desligar(void) {
    printf("Desligando aspersor\r\n");
    if (tipo_acionamento == INVERSOR) {
        motor_desligar_inversor();
    } else {
        motor_desligar_manual();
    }
    return pdOK;
}

void ligar_no_horario(uint8_t hh, uint8_t mm, uint8_t ss, uint16_t tempo) {
    struct tm data_hora = sntp_pegar_data_hora();
    if ((data_hora.tm_hour == hh) && (data_hora.tm_min == mm) && (data_hora.tm_sec == ss))
        aspersor_ligar(tempo);
}

static void vMotorTask(void *pvParameters) {
    while (1) {
        if (motor.estado == LIGADO) {
            for (uint16_t i = 0; i < (motor.tempo * 10); i++) {
                if (botao_esc()) {
                    delay_ms(100);
                    break;
                }
                delay_ms(100);
            }
        }

        delay_ms(10);
    }
    vTaskDelete(NULL);
}

static void vMainTask(void *pvParameters) {
    // Inicialização dos relés
    RELE_DESACIONA_DESL;
    RELE_ACIONA_DESL;
    RELE_SAIDA_INVERSOR_DESL;
    RELE_SELECAO_INVERSOR;

    // Seleciona comando manual. Ativa as botoeiras e
    // não permite acionamento pelo inversor
    if (tipo_acionamento == MANUAL)
        RELE_SELECAO_MANUAL;

    while (1) {
        uint16_t tempo_irrigacao = 150;  // Em segundos

        if (botao_ent()) {
            delay_ms(100);
            aspersor_ligar(tempo_irrigacao);
        } else if (botao_esc()) {
            delay_ms(100);
            aspersor_desligar();
        }

        uint8_t horarios[][3] = {
            {7, 0, 0},
            {8, 0, 0},
            {9, 0, 0},
            {10, 0, 0},
            {11, 0, 0},
            {12, 0, 0},
            {13, 0, 0},
            {14, 0, 0},
            {15, 0, 0},
            {16, 0, 0},
            {17, 0, 0}
        };

        uint8_t horarios_len = sizeof(horarios) / sizeof(horarios[0]);

        for (int i = 0; i < horarios_len; i++) {
            ligar_no_horario(horarios[i][0], horarios[i][1], horarios[i][2], tempo_irrigacao);
        }

        delay_ms(10);
    }
    vTaskDelete(NULL);
}

static void vMqttInitTask(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(semaph_wifi_con, portMAX_DELAY)) {
            printf("Conectado ao WiFi!\r\n");
            mqtt_start();
        }
    }
    vTaskDelete(NULL);
}

static void vMqttTxTask(void *pvParameters) {
    //char mensagem[50];

    if (xSemaphoreTake(semaph_mqtt_con, portMAX_DELAY)) {
        while (1) {
            // float temperatura = 20.0 + (float)rand() / (float)(RAND_MAX / 10.0);
            // sprintf(mensagem, "temperatura: %.2f", temperatura);
            // mqtt_enviar("sensores/temperatura", mensagem);
            delay_s(3);
        }
        vTaskDelete(NULL);
    }
}

error_t verifica_msg_mqtt_rx(mqtt_t mqtt_rx) {
    if (mqtt_rx.id != DISPOSITIVO_ID) {
        ESP_LOGE(TAG, "ID invalido!\r\n");
        return pdERROR;
    }

    if (strcmp(mqtt_rx.msg, "tche") == 0) {
        printf("mas tche, ai que eu me refiro\r\n");
        return pdOK;
    } else if (strcmp(mqtt_rx.msg, "ligar") == 0) {
        aspersor_ligar(150);
        printf("Enviando resposta ao servidor\r\n");
        mqtt_enviar("resposta/motor", "motor ligado");
        return pdOK;
    } else if (strcmp(mqtt_rx.msg, "desligar") == 0) {
        aspersor_desligar();
        printf("Enviando resposta ao servidor\r\n");
        mqtt_enviar("resposta/motor", "motor desligado");
        return pdOK;
    }

    return pdOK;
}

static void vMqttRxTask(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(semaph_mqtt_rx, portMAX_DELAY))
            verifica_msg_mqtt_rx(mqtt_receber());
    }
}

static void vSntpTask(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(semaph_sntp_con, portMAX_DELAY)) {
            sntp_start();
            while (1) {
                struct tm tempo = sntp_pegar_data_hora();
                char strftime_buf[64];

                strftime(strftime_buf, sizeof(strftime_buf), "%c", &tempo);
                // printf("Data/hora atual: %s\r\n", strftime_buf);

                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            vTaskDelete(NULL);
        }
    }
}

void flash_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void app_main(void) {
    flash_init();
    botao_init();
    i2c_master_init();
    inversor_init();
    rele_init();
    //wifi_init("AQUI_TEM_AGUA_PRO_CHIMARRAO", "masbahtche");
    wifi_init("ABRIGO", "12345678");

    semaph_motor_ligar = xSemaphoreCreateBinary();
    semaph_motor_desligar = xSemaphoreCreateBinary();

    xTaskCreate(vMqttInitTask,
                "vMqttInitTask",
                4096,
                NULL,
                tskIDLE_PRIORITY,
                xHandle_mqttInitTask);

    xTaskCreate(vSntpTask,
                "vSntpTask",
                4096,
                NULL,
                tskIDLE_PRIORITY,
                xHandle_sntpTask);

    xTaskCreate(vMqttRxTask,
                "vMqttRxTask",
                4096,
                NULL,
                tskIDLE_PRIORITY,
                xHandle_mqttRxTask);

    xTaskCreate(vMqttTxTask,
                "vMqttTxTask",
                4096,
                NULL,
                tskIDLE_PRIORITY,
                xHandle_mqttTxTask);

    xTaskCreate(vMainTask,
                "vMainTask",
                4096,
                NULL,
                tskIDLE_PRIORITY + 2,
                xHandle_mainTask);

    vTaskCreate(vMotorTask,
                "vMotorTask",
                4096,
                NULL,
                tskIDLE_PRIORITY,
                xHandle_motorTask);

    // xTaskCreate(vBme280Task,
    //             "vBme280Task",
    //             1024,
    //             NULL,
    //             tskIDLE_PRIORITY + 5,
    //             NULL);
}
