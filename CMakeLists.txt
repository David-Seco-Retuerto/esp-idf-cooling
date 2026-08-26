idf_component_register(
    SRCS
        "cooling.c"

    INCLUDE_DIRS
        "include"

    REQUIRES
        esp_driver_ledc

    PRIV_REQUIRES
        esp_driver_gpio
)