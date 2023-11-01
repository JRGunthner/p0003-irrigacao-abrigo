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
    uint16_t id;
} app_t;

typedef struct {
    uint8_t h;
    uint8_t m;
    uint8_t s;
} agenda_t;

#ifdef __cplusplus
}
#endif

#endif // __TYPES_H__
