#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "driver/gpio.h"

#define VERSAO 100

//#define MODO_ABRIGO
#define MODO_MORANGO

#ifdef MODO_ABRIGO
#define UART_PIN_TX  GPIO_NUM_11
#define UART_PIN_RX  GPIO_NUM_10
#define UART_PIN_RTS GPIO_NUM_9

#define SDA_PIN GPIO_NUM_1
#define SCL_PIN GPIO_NUM_2

#define RELE_1 GPIO_NUM_8
#define RELE_2 GPIO_NUM_19
#define RELE_3 GPIO_NUM_20
#define RELE_4 GPIO_NUM_3

#define BOTAO_ESC GPIO_NUM_14
#define BOTAO_ENT GPIO_NUM_13

#define LED_BR GPIO_NUM_5
#else // MODO_MORANGO
#define UART_PIN_TX  GPIO_NUM_11
#define UART_PIN_RX  GPIO_NUM_10
#define UART_PIN_RTS GPIO_NUM_9

#define SDA_PIN GPIO_NUM_1
#define SCL_PIN GPIO_NUM_2

#define RELE_1 GPIO_NUM_38
#define RELE_2 GPIO_NUM_37
#define RELE_3 GPIO_NUM_36
#define RELE_4 GPIO_NUM_35
#define RELE_5 GPIO_NUM_45
#define RELE_6 GPIO_NUM_48
#define RELE_7 GPIO_NUM_47
#define RELE_8 GPIO_NUM_21
#endif // MODO_ABRIGO

#ifdef __cplusplus
}
#endif

#endif // __MAIN_H__
