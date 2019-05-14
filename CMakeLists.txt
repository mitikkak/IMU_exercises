
cmake_minimum_required(VERSION 3.1)

set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(LCD_SRC ${PROJECT_ROOT}/LiquidCrystal_I2C)
set(ARDUINO_CMAKE_DIR ${PROJECT_ROOT}/arduino_cmake)
set(TIMERONE_SRC ${PROJECT_ROOT}/TimerOne)
set(WEB_SOCKETS_DIR ${PROJECT_ROOT}/arduinoWebSockets/src)
set(web_sockets_sources ${WEB_SOCKETS_DIR}/WebSocketsServer.cpp
                        ${WEB_SOCKETS_DIR}/WebSocketsClient.cpp
                        ${WEB_SOCKETS_DIR}/WebSockets.cpp
)

set(PCD8544_SRC ${PROJECT_ROOT}/Adafruit-PCD8544)
set(PCD8544_ESP8266_SRC ${PROJECT_ROOT}/Adafruit-PCD8544-Nokia-5110-LCD-library)
set(ADAFRUIT_GFX_SRC ${PROJECT_ROOT}/Adafruit-GFX-Library)
set(adafruit_sources ${PCD8544_SRC}/Adafruit_PCD8544.cpp
                     ${ADAFRUIT_GFX_SRC}/Adafruit_GFX.cpp
)
set(adafruit_esp8266_sources ${PCD8544_ESP8266_SRC}/Adafruit_PCD8544.cpp
                     ${ADAFRUIT_GFX_SRC}/Adafruit_GFX.cpp
)

add_subdirectory(${PROJECT_ROOT}/googletest)
add_subdirectory(${ARDUINO_CMAKE_DIR}/esp8266/core)
add_subdirectory(${PROJECT_ROOT}/MPU9250_MS5637_t3)
add_subdirectory(${PROJECT_ROOT}/MPU9250_lulu)
add_subdirectory(${PROJECT_ROOT}/sparkfun)
add_subdirectory(${PROJECT_ROOT}/sparkfun_dmp)
add_subdirectory(${PROJECT_ROOT}/mpu6050_dmp6)
