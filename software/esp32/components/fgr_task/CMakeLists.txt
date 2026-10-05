# Task related stuff implementation for the front garden railway
idf_component_register(SRCS "fgr_task.c"
                       INCLUDE_DIRS "api"
                       PRIV_REQUIRES fgr_util)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
