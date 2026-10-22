# TMC5160 stepper motor driver driver
idf_component_register(SRCS "tmc5160.c"
                       INCLUDE_DIRS "api"
                       PRIV_REQUIRES fgr_util esp_driver_gpio esp_driver_spi esp_timer)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
