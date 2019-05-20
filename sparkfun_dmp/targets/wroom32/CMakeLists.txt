
project(sparkfun_dmp_wroom32)

include(${ARDUINO_CMAKE_DIR}/esp32/wroom32.cmake)

set(ext_sources 
    ${sparkfun_dmp_lib_sources}
    ${esp32_wire_sources}
    #${adafruit_esp8266_sources}
    ${esp32_spi_sources}
    ${esp32_wifi_sources}
    ${web_sockets_sources}
)
set(EXT_SRC_DIR
    ${SPARKFUN_DMP_LIB_SRC}
    ${ESP32_WIRE_SRC_DIR}
    ${ADAFRUIT_GFX_SRC}
    ${PCD8544_ESP8266_SRC}
    ${ESP32_SPI_SRC_DIR}
    ${WEB_SOCKETS_DIR}
    ${ESP32_WIFI_SRC_DIR}
    ${ESP32_WIFI_CLIENT_SECURE_SRC_DIR}
)

set(all_sources ${sources} ${ext_sources})
build_targets_espwroom32("${all_sources}" "${EXT_SRC_DIR}")
