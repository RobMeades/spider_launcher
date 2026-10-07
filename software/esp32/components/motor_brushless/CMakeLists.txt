# Brushless motor driver
idf_component_register(SRCS "motor_brushless.c"
                       INCLUDE_DIRS "api"
                       PRIV_REQUIRES fgr_util driver esp_driver_gpio)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
