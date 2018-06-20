
cmake_minimum_required(VERSION 3.1)

set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(LCD_SRC ${PROJECT_ROOT}/LiquidCrystal_I2C)
set(ARDUINO_CMAKE_DIR ${PROJECT_ROOT}/arduino_cmake)

add_subdirectory(${PROJECT_ROOT}/googletest)
add_subdirectory(${ARDUINO_CMAKE_DIR}/esp8266/core)
add_subdirectory(${PROJECT_ROOT}/MPU9250_MS5637_t3)
add_subdirectory(${PROJECT_ROOT}/MPU9250_lulu)
