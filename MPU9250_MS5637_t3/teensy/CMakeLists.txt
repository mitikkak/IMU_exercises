
project(MPU9250_MS5637_t3_teensy)

include(${ARDUINO_CMAKE_DIR}/teensy/paths.cmake)
include(${ARDUINO_CMAKE_DIR}/teensy/tools.cmake)
include(${ARDUINO_CMAKE_DIR}/teensy/defs.cmake)
include(${ARDUINO_CMAKE_DIR}/teensy/core.cmake)
include(${ARDUINO_CMAKE_DIR}/teensy/deploy.cmake)
include(${ARDUINO_CMAKE_DIR}/teensy/libs.cmake)

add_executable(${PROJECT_NAME} ${sources})

target_link_libraries(${PROJECT_NAME} teensy_core teensy_libs)
target_compile_options(
    ${PROJECT_NAME} PRIVATE
    ${teensy_compiler_set_cpp}
)
target_include_directories(
    ${PROJECT_NAME} PRIVATE
    ${TEENSY_LCD_SRC}
    ${TEENSY_WIRE_SRC}
    ${TEENSY_SPI_SRC}
)

upload_teensy(${PROJECT_NAME} ${PROJECT_NAME})