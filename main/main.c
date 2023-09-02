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
#include "protocol_examples_common.h"
#include "bme280.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "inversor.h"
#include "sens_tpu.h"
#include "teclado.h"

#define TAG_SNTP "SNTP"

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

// Rotina WiFi //////////////////////////////////////////////////
static void obtain_time(void);
static void initialize_sntp(void);

void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG_SNTP, "Notification of a time synchronization event");
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
        if (botao_esc())
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

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    botao_init();

    // Initialize I2C parameters
    i2c_master_init();

    // Read the data from BME280 sensor
    xTaskCreate(Publisher_Task, "Publisher_Task", 1024 * 5, NULL, 5, NULL);

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
        if (botao_ent()) {
            // RELE_ACIONA_LIGA;
            // RELE_DESACIONA_DESL;
            // vTaskDelay(100 / portTICK_PERIOD_MS);
            // RELE_ACIONA_DESL;

            aciona_aspersor(120);
        } else if (botao_esc()) {
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
            if ((data.tm_hour) >= 0)
                gpio_set_level(LED_BR, 1);
            else
                gpio_set_level(LED_BR, 0);
            //printf("%02d:%02d:%02d, Temperatura: %s, Pressão: %s, Umidade: %s\r\n", data.tm_hour, data.tm_min, data.tm_sec, temperature, pressure, humidity);
            printf("%02d:%02d:%02d\r\n", data.tm_hour, data.tm_min, data.tm_sec);
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
        ESP_LOGI(TAG_SNTP, "Esperando pela hora do servidor... (%d/%d)", retry, retry_count);
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
