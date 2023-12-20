#include "teclado.h"
#include "driver/gpio.h"

void teclado_init(void) {
#ifdef MODO_ABRIGO
    gpio_reset_pin(BOTAO_ESC);
    gpio_set_direction(BOTAO_ESC, GPIO_MODE_INPUT);

    gpio_reset_pin(BOTAO_ENT);
    gpio_set_direction(BOTAO_ENT, GPIO_MODE_INPUT);
#endif
}

uint8_t teclado_btn_ent(void) {
#ifdef MODO_ABRIGO
    return !gpio_get_level(BOTAO_ENT);
#else
    return 0;
#endif
}

uint8_t teclado_btn_esc(void) {
#ifdef MODO_ABRIGO
    return !gpio_get_level(BOTAO_ESC);
#else
    return 0;
#endif
}
