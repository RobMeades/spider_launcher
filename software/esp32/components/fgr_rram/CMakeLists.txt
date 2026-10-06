# Retained RAM implementation for the front garden railway
idf_component_register(SRCS "fgr_rram.c"
                       INCLUDE_DIRS "api")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
