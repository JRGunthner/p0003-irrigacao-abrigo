#ifndef __RELE_H__
#define __RELE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "main.h"

void rele_init(void);
void rele_start_stop(uint8_t rele, bool estado);

#define RELE_DESACIONA_LIGA      rele_start_stop(RELE_1, 0)
#define RELE_DESACIONA_DESL      rele_start_stop(RELE_1, 1)
#define RELE_ACIONA_LIGA         rele_start_stop(RELE_2, 0)
#define RELE_ACIONA_DESL         rele_start_stop(RELE_2, 1)
#define RELE_SAIDA_INVERSOR_LIGA rele_start_stop(RELE_3, 0)
#define RELE_SAIDA_INVERSOR_DESL rele_start_stop(RELE_3, 1)
#define RELE_SELECAO_MANUAL      rele_start_stop(RELE_4, 0)
#define RELE_SELECAO_INVERSOR    rele_start_stop(RELE_4, 1)

#ifdef __cplusplus
}
#endif

#endif // __RELE_H__
