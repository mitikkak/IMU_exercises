
project(accelerometer_tilt_wroom32)

include(${ARDUINO_CMAKE_DIR}/esp32/wroom32.cmake)

set(ext_sources 
    ${sparkfun_dmp_lib_sources}
    ${esp32_wire_sources}
    ${esp32_spi_sources}
    ${esp32_wifi_sources}
    ${web_sockets_sources}
    ${esp32_async_udp_sources}
    ${esp32_ota_sources}
    ${esp32_update_sources}
    ${esp32_mdns_sources}
    ${adafruit_esp32_sources}
    ${adxl345_sources}
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
    ${ESP32_ASYNC_UDP_SRC_DIR}
    ${ESP32_OTA_SRC_DIR}
    ${ESP32_UPDATE_SRC_DIR}
    ${ESP32_MDNS_SRC_DIR}
    ${PCD8544_ESP32_SRC}
    ${ADXL345_SRC}
)

set(all_sources ${sources} ${ext_sources})
build_targets_espwroom32("${all_sources}" "${EXT_SRC_DIR}")
