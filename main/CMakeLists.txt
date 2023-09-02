set(PROJECT_NAME "p0003-irrigacao-abrigo")

idf_component_register(
    SRCS
        "bme280.c"
        "main.c"
        "motor/modbus_params.c"
        "motor/inversor.c"
    INCLUDE_DIRS "." "./motor"
)
