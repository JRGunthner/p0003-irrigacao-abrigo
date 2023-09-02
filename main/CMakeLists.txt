set(PROJECT_NAME "modbus_master")

idf_component_register(
    SRCS "bme280.c" "main.c" "modbus_params.c"
    INCLUDE_DIRS "."
)
