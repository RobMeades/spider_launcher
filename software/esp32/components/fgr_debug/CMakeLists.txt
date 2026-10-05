# Debug utilities for the front garden railway
idf_component_register(SRCS "fgr_debug.c"
                       INCLUDE_DIRS "api"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_spi esp_system esp_partition espcoredump esp_app_format mbedtls fgr_util fgr_ws2812 fgr_task fgr_nvs)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)