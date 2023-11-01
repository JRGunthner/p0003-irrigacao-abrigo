#include "teclado.h"
#include "driver/gpio.h"

void teclado_init(void) {
    gpio_reset_pin(BOTAO_ESC);
    gpio_set_direction(BOTAO_ESC, GPIO_MODE_INPUT);

    gpio_reset_pin(BOTAO_ENT);
    gpio_set_direction(BOTAO_ENT, GPIO_MODE_INPUT);
}

uint8_t teclado_btn_ent(void) {
    return !gpio_get_level(BOTAO_ENT);
}

uint8_t teclado_btn_esc(void) {
    return !gpio_get_level(BOTAO_ESC);
}
