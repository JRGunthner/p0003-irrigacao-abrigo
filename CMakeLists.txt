# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS motor)
set(EXTRA_COMPONENT_DIRS sens_tpu)
set(EXTRA_COMPONENT_DIRS teclado)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p0003-irrigacao-abrigo)
