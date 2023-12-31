#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi.h"

#define WIFI_MAX_TENTATIVAS_RECONEXAO 5

#define WIFI_BIT_CONECTADO  BIT0
#define WIFI_BIT_FALHA      BIT1

#define TAG "WIFI"

static EventGroupHandle_t wifi_grupo_eventos;
static uint16_t cont_tentativas = 0;

static void event_handler(void *arg, esp_event_base_t evento_base, int32_t evento_id, void *evento_dados) {
    if ((evento_base == WIFI_EVENT) && (evento_id == WIFI_EVENT_STA_START)) {
        esp_wifi_connect();
    } else if ((evento_base == WIFI_EVENT) && (evento_id == WIFI_EVENT_STA_DISCONNECTED)) {
        if (cont_tentativas < WIFI_MAX_TENTATIVAS_RECONEXAO) {
            esp_wifi_connect();
            cont_tentativas++;
            ESP_LOGI(TAG, "Tentando reconectar a rede WiFi...");
        } else {
            xEventGroupSetBits(wifi_grupo_eventos, WIFI_BIT_FALHA);
        }
        ESP_LOGI(TAG, "Falha ao conectar na rede WiFi.");
    } else if ((evento_base == IP_EVENT) && (evento_id == IP_EVENT_STA_GOT_IP)) {
        ip_event_got_ip_t *evento = (ip_event_got_ip_t *)evento_dados;
        ESP_LOGI(TAG, "Endereco IP recebido:" IPSTR, IP2STR(&evento->ip_info.ip));
        cont_tentativas = 0;
        xEventGroupSetBits(wifi_grupo_eventos, WIFI_BIT_CONECTADO);
        xSemaphoreGive(semaph_wifi_con);
        xSemaphoreGive(semaph_sntp_con);
    }
}

void wifi_init(const char *wifi_ssid, const char *wifi_pass) {
    semaph_wifi_con = xSemaphoreCreateBinary();
    semaph_sntp_con = xSemaphoreCreateBinary();
    wifi_grupo_eventos = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t config = {.sta = {.ssid = "", .password = ""}};

    memcpy(config.sta.ssid, wifi_ssid, strlen(wifi_ssid));
    memcpy(config.sta.password, wifi_pass, strlen(wifi_pass));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &config));

    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(wifi_grupo_eventos, WIFI_BIT_CONECTADO | WIFI_BIT_FALHA,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_BIT_CONECTADO) {
        ESP_LOGI(TAG, "Conectado na rede WiFi: %s, senha: %s", config.sta.ssid, config.sta.password);
    } else if (bits & WIFI_BIT_FALHA) {
        ESP_LOGI(TAG, "Falha ao conectar na rede WiFi: %s, senha: %s", config.sta.ssid, config.sta.password);
    } else {
        ESP_LOGE(TAG, "Evento inesperado.");
    }
}
