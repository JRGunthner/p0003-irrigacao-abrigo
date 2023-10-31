#ifndef __INVERSOR_H__
#define __INVERSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "main.h"

typedef enum {
    DESLIGADO = 0,
    LIGADO,
    DESLIGANDO,
    LIGANDO
} estado_motor_t;

typedef enum {
    NENHUMA = 0,
    DESLIGAR,
    LIGAR
} acao_motor_t;

typedef struct {
    estado_motor_t estado;
    acao_motor_t acao;
    uint16_t rpm;
    uint16_t tempo;
} motor_t;

extern motor_t motor;

esp_err_t inversor_init(void);
void inversor_velocidade_motor(uint16_t rpm);
void inversor_desligar_motor(void);
void inversor_ligar_motor(void);

#ifdef __cplusplus
}
#endif

#endif // __INVERSOR_H__
