// Sensor de temperatura, pressão atm e umidade do ar
#ifndef __SENS_TPU_H__
#define __SENS_TPU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "main.h"

void i2c_master_init(void);
void Publisher_Task(void *params);

#ifdef __cplusplus
}
#endif

#endif // __SENS_TPU_H__
