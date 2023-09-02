#ifndef __INVERSOR_H__
#define __INVERSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "main.h"

esp_err_t master_init(void);
void motor_definir_velocidade(uint16_t rpm);
void motor_desliga(void);
void motor_liga(void);

#ifdef __cplusplus
}
#endif

#endif // __INVERSOR_H__
