#ifndef __WIFI_H__
#define __WIFI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <string.h>

xSemaphoreHandle semaph_wifi_con;
xSemaphoreHandle semaph_sntp_con;

void wifi_init(const char *wifi_ssid, const char *wifi_pass);

#ifdef __cplusplus
}
#endif

#endif // __WIFI_H__
