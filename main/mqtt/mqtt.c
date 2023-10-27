#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "mqtt.h"

#define TAG "MQTT"

esp_mqtt_client_handle_t client;
static mqtt_t mqtt;

// Analisar a string JSON e retorna uma struct
static mqtt_t mqtt_parse_json_str(const char *jsonString) {
    cJSON *msg_json = cJSON_Parse(jsonString);
    if (msg_json == NULL) {
        fprintf(stderr, "Erro ao analisar JSON.\n");
        exit(1);
    }

    mqtt_t msg_struct;
    cJSON *id = cJSON_GetObjectItem(msg_json, "id");
    cJSON *msg = cJSON_GetObjectItem(msg_json, "msg");

    if (id && msg) {
        msg_struct.id = id->valueint;
        memcpy(msg_struct.msg, msg->valuestring, sizeof(msg_struct.msg));
    } else {
        fprintf(stderr, "Campos de JSON ausentes ou invalidos.\n");
        // TODO: tratar campos faltantes
    }

    cJSON_Delete(msg_json);
    return msg_struct;
}

static void mqtt_log_erro(const char *message, int error_code) {
    if (error_code != 0)
        ESP_LOGE(TAG, "Ultimo erro %s: 0x%x", message, error_code);
}

static void mqtt_tratar_msg_rx(const char *mensagem, uint16_t len) {
    char *nova_msg = "";
    memcpy(nova_msg, mensagem, len);
    mqtt = mqtt_parse_json_str(nova_msg);
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            xSemaphoreGive(semaph_mqtt_con);
            msg_id = esp_mqtt_client_subscribe(client, "servidor/resposta", 0);
            ESP_LOGI(TAG, "Mensagem recebida, ID: %d\r\n", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED: ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED"); break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPICO=%.*s\r\n", event->topic_len, event->topic);
            printf("RxD=%.*s\r\n", event->data_len, event->data);
            mqtt_tratar_msg_rx(event->data, event->data_len);
            xSemaphoreGive(semaph_mqtt_rx);

            // se event->data for igual a ligar_motor, printf(ligar motor)
            // if (strncmp(event->data, "ligar_motor", event->data_len) == 0) {
            //     printf("Ligar motor\r\n");
            //     mqtt_liga_motor();
            // }
            // if (strncmp(event->data, "desligar_motor", event->data_len) == 0) {
            //     printf("Desligar motor\r\n");
            //     mqtt_desliga_motor();
            // }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                mqtt_log_erro("Reportado pelo esp-tls", event->error_handle->esp_tls_last_esp_err);
                mqtt_log_erro("Reportado pela stack tls", event->error_handle->esp_tls_stack_err);
                mqtt_log_erro("Capturado como numero do erro do soquete de transporte",
                              event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Ultimo erro (%s)",
                         strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default: ESP_LOGI(TAG, "Outro evento id:%d", event->event_id); break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id,
                               void *event_data) {
    ESP_LOGD(TAG, "Evento despachado do loop de eventos base=%s, evento_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

void mqtt_start(void) {
    semaph_mqtt_con = xSemaphoreCreateBinary();
    semaph_mqtt_rx = xSemaphoreCreateBinary();

    esp_mqtt_client_config_t mqtt_cfg = {
        .host = "irrigacao.jgtche.com.br",
        .port = 1883,
        .username = "mastche",
        .password = "123456789",
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

void mqtt_enviar(char *topico, char *mensagem) {
    int msg_id = esp_mqtt_client_publish(client, topico, mensagem, 0, 1, 0);
    printf("Mensagem enviada, ID: %d\r\n", msg_id);
}

mqtt_t mqtt_receber(void) {
    return mqtt;
}
