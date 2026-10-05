# Utilities implementation for the front garden railway
idf_component_register(SRCS "fgr_util.c"
                       INCLUDE_DIRS "api")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
