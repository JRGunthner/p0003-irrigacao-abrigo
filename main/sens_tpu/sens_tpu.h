// Sensor de temperatura, pressão atm e umidade do ar
#ifndef __SENS_TPU_H__
#define __SENS_TPU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "main.h"

typedef struct {
    char temp[12];
    char humi[10];
    char pres[10];
} sens_tpu_t;

void i2c_master_init(void);
void vSensTpuTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // __SENS_TPU_H__
