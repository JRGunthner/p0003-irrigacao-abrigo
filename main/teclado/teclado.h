#ifndef _TECLADO_H_
#define _TECLADO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "main.h"

void botao_init(void);
uint8_t botao_ent(void);
uint8_t botao_esc(void);

#ifdef __cplusplus
}
#endif

#endif // _TECLADO_H_
