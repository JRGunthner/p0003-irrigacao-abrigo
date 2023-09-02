#ifndef __INVERSOR_H__
#define __INVERSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "driver/gpio.h"

#define UART_PIN_TX  GPIO_NUM_11
#define UART_PIN_RX  GPIO_NUM_10
#define UART_PIN_RTS GPIO_NUM_9

void inversor_init(void);

esp_err_t master_init(void);
void motor_definir_velocidade(uint16_t rpm);
void motor_desliga(void);
void motor_liga(void);

#ifdef __cplusplus
}
#endif

#endif // __INVERSOR_H__
