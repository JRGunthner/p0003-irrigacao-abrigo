#ifndef __WIFI_H__
#define __WIFI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <string.h>

xSemaphoreHandle semaph_con_wifi;

void wifi_init(void);

#ifdef __cplusplus
}
#endif

#endif // __WIFI_H__
