#ifndef __INVERSOR_H__
#define __INVERSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "main.h"

esp_err_t inversor_init(void);
void inversor_velocidade_motor(uint16_t rpm);
void inversor_desligar_motor(void);
void inversor_ligar_motor(void);

#ifdef __cplusplus
}
#endif

#endif // __INVERSOR_H__
