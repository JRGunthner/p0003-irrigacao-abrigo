set(PROJECT_NAME "p0003-irrigacao-abrigo")

idf_component_register(
    SRCS
        "main.c"
        "motor/modbus_params.c"
        "motor/inversor.c"
        "mqtt/mqtt.c"
        "sens_tpu/bme280.c"
        "sens_tpu/sens_tpu.c"
        "teclado/teclado"
        "wifi/wifi.c"
    INCLUDE_DIRS
        "."
        "./motor"
        "./mqtt"
        "./sens_tpu"
        "./teclado"
        "./wifi"
)
