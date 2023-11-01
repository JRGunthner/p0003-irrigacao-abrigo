#ifndef __TYPES_H__
#define __TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    pdOK,
    pdERROR,
    pdUNDEF
} error_t;

typedef enum {
    INVERSOR = 0,
    MANUAL
} tipo_acionamento_t;

typedef struct {
    tipo_acionamento_t acionamento;
    struct {
        char *ssid;
        char *senha;
    } wifi;
} app_t;

#ifdef __cplusplus
}
#endif

#endif // __TYPES_H__
