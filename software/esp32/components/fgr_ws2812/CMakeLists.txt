# WS2812 LED driver for the front garden railway
idf_component_register(SRCS "fgr_ws2812.c"
                       INCLUDE_DIRS "api"
                       PRIV_REQUIRES esp_driver_spi fgr_util fgr_task)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)