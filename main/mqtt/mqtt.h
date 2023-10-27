#ifndef __MQTT_H__
#define __MQTT_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

typedef struct {
    uint32_t id;
    char msg[256];
} mqtt_t;

xSemaphoreHandle semaph_mqtt_con;
xSemaphoreHandle semaph_mqtt_rx;

void mqtt_start(void);
void mqtt_enviar(char *topico, char *mensagem);
mqtt_t mqtt_receber(void);

#endif  // __MQTT_H__
