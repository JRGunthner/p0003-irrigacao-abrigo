set(PROJECT_NAME "p0003-irrigacao-abrigo")

idf_component_register(
    SRCS
        "main.c"
        "motor/modbus_params.c"
        "motor/inversor.c"
        "mqtt/mqtt.c"
        "rele/rele.c"
        "sens_tpu/bme280.c"
        "sens_tpu/sens_tpu.c"
        "sntp/sntp.c"
        "teclado/teclado"
        "wifi/wifi.c"
    INCLUDE_DIRS
        "."
        "./motor"
        "./mqtt"
        "./rele"
        "./sens_tpu"
        "./sntp"
        "./teclado"
        "./wifi"
)
