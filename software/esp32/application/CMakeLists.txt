# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Set default target ONLY if not overridden by environment or command line
if(NOT DEFINED ENV{IDF_TARGET} AND NOT DEFINED IDF_TARGET)
    set(IDF_TARGET "esp32s3" CACHE STRING "Target chip" FORCE)
endif()

# DESTROY THE CMake CACHE - it lies, it cheats, it steals hours of your life
# This forces a clean configuration EVERY time you run CMake
if(EXISTS "${CMAKE_BINARY_DIR}/CMakeCache.txt")
    file(REMOVE "${CMAKE_BINARY_DIR}/CMakeCache.txt")
    file(REMOVE_RECURSE "${CMAKE_BINARY_DIR}/CMakeFiles")
endif()

# Add compilation options globally for all targets
add_compile_options(-Wall -Wextra -Werror)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS "../components")
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(spider_launcher)
