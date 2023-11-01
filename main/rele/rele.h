#ifndef __RELE_H__
#define __RELE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "main.h"

typedef enum {
    DESL = 0,
    LIGA
} rele_t;

void rele_init(void);
void rele_liga_desl(uint8_t rele, rele_t estado);

#define RELE_DESACIONA_LIGA      rele_liga_desl(RELE_1, DESL)
#define RELE_DESACIONA_DESL      rele_liga_desl(RELE_1, LIGA)
#define RELE_ACIONA_LIGA         rele_liga_desl(RELE_2, DESL)
#define RELE_ACIONA_DESL         rele_liga_desl(RELE_2, LIGA)
#define RELE_SAIDA_INVERSOR_LIGA rele_liga_desl(RELE_3, DESL)
#define RELE_SAIDA_INVERSOR_DESL rele_liga_desl(RELE_3, LIGA)
#define RELE_SELECAO_MANUAL      rele_liga_desl(RELE_4, DESL)
#define RELE_SELECAO_INVERSOR    rele_liga_desl(RELE_4, LIGA)

#ifdef __cplusplus
}
#endif

#endif // __RELE_H__
