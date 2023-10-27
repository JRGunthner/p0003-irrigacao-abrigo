#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "nvs_flash.h"
#include "bme280.h"

#include "inversor.h"
#include "sens_tpu.h"
#include "teclado.h"
#include "wifi.h"
#include "mqtt.h"
#include "sntp.h"

xTaskHandle xHandle_mqttInitTask = NULL;
xTaskHandle xHandle_mqttRxTask = NULL;
xTaskHandle xHandle_mqttTxTask = NULL;
xTaskHandle xHandle_mainTask = NULL;
xTaskHandle xHandle_sntpTask = NULL;

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

void aciona_aspersor(uint16_t tempo) {
    RELE_SAIDA_INVERSOR_LIGA;
    delay_ms(300);
    motor_liga();
    motor_definir_velocidade(3470);

    for (uint16_t i = 0; i < (tempo * 10); i++) {
        if (botao_esc()) {
            delay_ms(100);
            break;
        }
        delay_ms(100);
    }

    delay_ms(100);
    motor_desliga();
    delay_s(6);
    RELE_SAIDA_INVERSOR_DESL;
    return;
}

void ligar_no_horario(uint8_t hh, uint8_t mm, uint8_t ss, uint16_t tempo) {
    struct tm data_hora = sntp_pegar_data_hora();
    if ((data_hora.tm_hour == hh) && (data_hora.tm_min == mm) && (data_hora.tm_sec == ss))
        aciona_aspersor(tempo);
}

static void vMainTask(void *pvParameters) {
    // Inicialização dos relés
    RELE_DESACIONA_DESL;
    RELE_ACIONA_DESL;
    RELE_SAIDA_INVERSOR_DESL;
    RELE_SELECAO_INVERSOR;

    // Seleciona comando manual. Ativa as botoeiras e
    // não permite acionamento pelo inversor
    // RELE_SELECAO_MANUAL;

    while (1) {
        uint16_t tempo_irrigacao = 150;  // Em segundos

        if (botao_ent()) {
            delay_ms(100);
            // RELE_ACIONA_LIGA;
            // RELE_DESACIONA_DESL;
            // delay_ms(100);
            // RELE_ACIONA_DESL;

            aciona_aspersor(tempo_irrigacao);
        } else if (botao_esc()) {
            delay_ms(100);
            // RELE_DESACIONA_LIGA;
            // RELE_ACIONA_DESL;
            // delay_ms(100);
            // RELE_DESACIONA_DESL;

            motor_desliga();
            delay_s(6);
            RELE_SAIDA_INVERSOR_DESL;
        }

        ligar_no_horario(7, 0, 0, tempo_irrigacao);
        ligar_no_horario(8, 0, 0, tempo_irrigacao);
        ligar_no_horario(9, 0, 0, tempo_irrigacao);
        ligar_no_horario(10, 0, 0, tempo_irrigacao);
        ligar_no_horario(11, 0, 0, tempo_irrigacao);
        ligar_no_horario(12, 0, 0, tempo_irrigacao);
        ligar_no_horario(13, 0, 0, tempo_irrigacao);
        ligar_no_horario(14, 0, 0, tempo_irrigacao);
        ligar_no_horario(15, 0, 0, tempo_irrigacao);
        ligar_no_horario(16, 0, 0, tempo_irrigacao);
        ligar_no_horario(17, 0, 0, tempo_irrigacao);
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

static void vMqttRxTask(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(semaph_mqtt_rx, portMAX_DELAY)) {
            mqtt_t mqtt_rx = mqtt_receber();
            printf("ID: %d\r\n", mqtt_rx.id);
            printf("Msg: %s\r\n", mqtt_rx.msg);
        }
    }
    vTaskDelete(NULL);
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
    wifi_init("Visitantes", "12345678");

    xTaskCreate(vMqttInitTask, "vMqttInitTask", 4096, NULL, tskIDLE_PRIORITY, xHandle_mqttInitTask);

    xTaskCreate(vMainTask,
                "vMainTask",
                4096,
                NULL,
                tskIDLE_PRIORITY + 2,
                xHandle_mainTask);

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

    // xTaskCreate(vBme280Task,
    //             "vBme280Task",
    //             1024,
    //             NULL,
    //             tskIDLE_PRIORITY + 5,
    //             NULL);
}
