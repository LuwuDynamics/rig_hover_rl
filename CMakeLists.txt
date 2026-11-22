# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Add this line to disable the specific warning
add_compile_options(-Wno-missing-field-initializers)

set(IDF_TARGET "esp32s3" CACHE STRING "Hover target")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)

set(BOARD_DIR "hover")
set(PROJECT_BIN_NAME "rig-hover")
set(BOARD_VERSION_FILE "${CMAKE_SOURCE_DIR}/main/boards/hover/version.txt")
file(STRINGS "${BOARD_VERSION_FILE}" PROJECT_VER LIMIT_COUNT 1)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${BOARD_VERSION_FILE}")
project(${PROJECT_BIN_NAME})

# 自定义命令：显示当前编译的板子和版本信息
# 用法: idf.py board-info
add_custom_target(board-info
    COMMAND ${CMAKE_COMMAND} -E echo "========================================"
    COMMAND ${CMAKE_COMMAND} -E echo "  Board:      ${PROJECT_BIN_NAME}"
    COMMAND ${CMAKE_COMMAND} -E echo "  Version:    ${PROJECT_VER}"
    COMMAND ${CMAKE_COMMAND} -E echo "  Chip:       ${IDF_TARGET}"
    COMMAND ${CMAKE_COMMAND} -E echo "  ESP-IDF:    $ENV{IDF_PATH}"
    COMMAND ${CMAKE_COMMAND} -E echo "  Ver File:   main/boards/${BOARD_DIR}/version.txt"
    COMMAND ${CMAKE_COMMAND} -E echo "========================================"
)
