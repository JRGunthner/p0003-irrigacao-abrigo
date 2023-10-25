#ifndef __SNTP_H__
#define __SNTP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

void sntp_start(void);
struct tm sntp_pegar_data_hora(void);

#ifdef __cplusplus
}
#endif

#endif // __SNTP_H__
