# Motor driver
idf_component_register(SRCS "motor.c"
                       INCLUDE_DIRS "api"
                       PRIV_REQUIRES fgr_util driver esp_driver_gpio esp_timer)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
