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

#include "driver/gpio.h"
#include "nvs_flash.h"
#include "bme280.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "inversor.h"
#include "sens_tpu.h"
#include "teclado.h"
#include "wifi.h"
#include "mqtt.h"

xSemaphoreHandle conexao_wifi_semaphore;
xSemaphoreHandle conexaoMqttSemaphore;

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

#define RELE_DESACIONA_LIGA rele_start_stop(RELE_1, 0)
#define RELE_DESACIONA_DESL rele_start_stop(RELE_1, 1)
#define RELE_ACIONA_LIGA rele_start_stop(RELE_2, 0)
#define RELE_ACIONA_DESL rele_start_stop(RELE_2, 1)
#define RELE_SAIDA_INVERSOR_LIGA rele_start_stop(RELE_3, 0)
#define RELE_SAIDA_INVERSOR_DESL rele_start_stop(RELE_3, 1)
#define RELE_SELECAO_MANUAL   rele_start_stop(RELE_4, 0)
#define RELE_SELECAO_INVERSOR rele_start_stop(RELE_4, 1)

void delay_s(uint16_t segundos) {
    vTaskDelay((segundos * 1000) / portTICK_PERIOD_MS);
}

void delay_ms(uint16_t milisegundos) {
    vTaskDelay(milisegundos / portTICK_PERIOD_MS);
}

void aciona_aspersor (uint16_t tempo) {
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

struct tm data;

void ligar_no_horario(uint8_t hh, uint8_t mm, uint8_t ss, uint16_t tempo) {
    if ((data.tm_hour == hh) && (data.tm_min == mm) && (data.tm_sec == ss))
        aciona_aspersor(tempo);
}

void mqtt_liga_motor(void) {
    delay_ms(100);
    aciona_aspersor(150);
}

void mqtt_desliga_motor(void) {
    motor_desliga();
    delay_s(6);
    RELE_SAIDA_INVERSOR_DESL;
}

void main_task(void *params) {
    uint8_t s_ant = 0;

    // Inicialização dos relés
    RELE_DESACIONA_DESL;
    RELE_ACIONA_DESL;
    RELE_SAIDA_INVERSOR_DESL;
    RELE_SELECAO_INVERSOR;

    // Seleciona comando manual. Ativa as botoeiras e
    // não permite acionamento pelo inversor
    //RELE_SELECAO_MANUAL;

    while (1) {
        if (botao_ent()) {
            delay_ms(100);
            // RELE_ACIONA_LIGA;
            // RELE_DESACIONA_DESL;
            // delay_ms(100);
            // RELE_ACIONA_DESL;

            aciona_aspersor(150);
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

        delay_ms(100);

        time_t tt = time(NULL);
        data = *gmtime(&tt);

        if (data.tm_sec != s_ant) {
            if ((data.tm_hour) >= 0)
                gpio_set_level(LED_BR, 1);
            else
                gpio_set_level(LED_BR, 0);
            //printf("%02d:%02d:%02d, Temperatura: %s, Pressão: %s, Umidade: %s\r\n", data.tm_hour, data.tm_min, data.tm_sec, temperature, pressure, humidity);
            printf("%02d:%02d:%02d\r\n", data.tm_hour, data.tm_min, data.tm_sec);
            s_ant = data.tm_sec;
        }

        uint16_t tempo_irrigacao = 150; // Em segundos

        //ligar_no_horario( 0, 0, 15, tempo_irrigacao);
        ligar_no_horario( 1, 0, 0, tempo_irrigacao);
        ligar_no_horario( 2, 0, 0, tempo_irrigacao);
        ligar_no_horario( 3, 0, 0, tempo_irrigacao);
        ligar_no_horario( 4, 0, 0, tempo_irrigacao);
        ligar_no_horario( 5, 0, 0, tempo_irrigacao);
        ligar_no_horario( 6, 0, 0, tempo_irrigacao);
        ligar_no_horario( 7, 0, 0, tempo_irrigacao);
        ligar_no_horario( 8, 0, 0, tempo_irrigacao);
        ligar_no_horario( 9, 0, 0, tempo_irrigacao);
        ligar_no_horario( 10, 0, 0, tempo_irrigacao);
        ligar_no_horario( 11, 0, 0, tempo_irrigacao);
    }
}

void conectadoWifi(void *params) {
    while (1) {
        if (xSemaphoreTake(conexao_wifi_semaphore, portMAX_DELAY)) {
            printf("Conectado ao WiFi!\r\n");
            mqtt_start();
        }
    }
    vTaskDelete(NULL);
}

void trataComunicacaoComServidor(void *params) {
    char mensagem[50];

    if (xSemaphoreTake(conexaoMqttSemaphore, portMAX_DELAY)) {
        while (1) {
            float temperatura = 20.0 + (float)rand()/(float)(RAND_MAX/10.0);
            sprintf(mensagem, "temperatura: %.2f", temperatura);
            mqtt_envia_mensagem("sensores/temperatura", mensagem);
            delay_s(3);
        }
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    botao_init();

    i2c_master_init();
    ESP_ERROR_CHECK(master_init());
    rele_init();

    conexao_wifi_semaphore = xSemaphoreCreateBinary();
    conexaoMqttSemaphore = xSemaphoreCreateBinary();
    wifi_start();

    xTaskCreate(&conectadoWifi, "Conexao ao MQTT", 4096, NULL, 1, NULL);
    //xTaskCreate(&trataComunicacaoComServidor, "Comunicacao com o Broker", 4096, NULL, 1, NULL);

    // Read the data from BME280 sensor
    //xTaskCreate(Publisher_Task, "Publisher_Task", 1024, NULL, 5, NULL);

    xTaskCreate(&main_task, "Funcao principal", 4096, NULL, 1, NULL);

    /* TODO: fazer nova task do sntp
    
    sntp_start();

    while (1) {
        struct tm tempo = pegar_data_hora();
        char strftime_buf[64];

        strftime(strftime_buf, sizeof(strftime_buf), "%c", &tempo);
        printf("Data/hora atual: %s\r\n", strftime_buf);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    */
}
