#ifndef __MQTT_H__
#define __MQTT_H__

void mqtt_start(void);
void mqtt_envia_mensagem(char *topico, char *mensagem);

extern void mqtt_liga_motor(void);
extern void mqtt_desliga_motor(void);

#endif // __MQTT_H__
