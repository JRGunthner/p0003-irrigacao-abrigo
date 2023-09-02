set(PROJECT_NAME "p0003-irrigacao-abrigo")

idf_component_register(
    SRCS
        "main.c"
        "motor/modbus_params.c"
        "motor/inversor.c"
        "sens_tpu/bme280.c"
        "sens_tpu/sens_tpu.c"
        "teclado/teclado"
    INCLUDE_DIRS
        "."
        "./motor"
        "./sens_tpu"
        "./teclado"
)
