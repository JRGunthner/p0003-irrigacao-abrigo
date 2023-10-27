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

xSemaphoreHandle semaph_con_mqtt;

void mqtt_start(void);
void mqtt_enviar_mensagem(char *topico, char *mensagem);

extern void mqtt_liga_motor(void);
extern void mqtt_desliga_motor(void);

#endif // __MQTT_H__
