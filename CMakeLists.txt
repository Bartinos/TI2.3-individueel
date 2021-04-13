# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# include(FetchContent)
# FetchContent_Declare(
#   espidflib
#   GIT_REPOSITORY https://github.com/UncleRus/esp-idf-lib.git
# )
# FetchContent_MakeAvailable(espidflib)

include($ENV{ADF_PATH}/CMakeLists.txt)
list(APPEND EXTRA_COMPONENT_DIRS ${espidflib_SOURCE_DIR}/components)
list(APPEND EXTRA_COMPONENT_DIRS ./components)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ti23-b1)