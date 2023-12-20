#include "driver/gpio.h"
#include "rele.h"

void rele_init(void) {
#ifdef MODO_ABRIGO
    gpio_reset_pin(RELE_1);
    gpio_set_direction(RELE_1, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_1, 1);

    gpio_reset_pin(RELE_2);
    gpio_set_direction(RELE_2, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_2, 1);

    gpio_reset_pin(RELE_3);
    gpio_set_direction(RELE_3, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_3, 1);

    gpio_reset_pin(RELE_4);
    gpio_set_direction(RELE_4, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_4, 1);

    gpio_reset_pin(LED_BR);
    gpio_set_direction(LED_BR, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_BR, 0);
#else
    gpio_reset_pin(RELE_1);
    gpio_set_direction(RELE_1, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_1, 1);

    gpio_reset_pin(RELE_2);
    gpio_set_direction(RELE_2, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_2, 1);

    gpio_reset_pin(RELE_3);
    gpio_set_direction(RELE_3, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_3, 1);

    gpio_reset_pin(RELE_4);
    gpio_set_direction(RELE_4, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_4, 1);
    
    gpio_reset_pin(RELE_5);
    gpio_set_direction(RELE_5, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_5, 1);

    gpio_reset_pin(RELE_6);
    gpio_set_direction(RELE_6, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_6, 1);

    gpio_reset_pin(RELE_7);
    gpio_set_direction(RELE_7, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_7, 1);

    gpio_reset_pin(RELE_8);
    gpio_set_direction(RELE_8, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_8, 1);
#endif
}

void rele_liga_desl(uint8_t rele, rele_t estado) {
    gpio_set_level(rele, estado);
}
