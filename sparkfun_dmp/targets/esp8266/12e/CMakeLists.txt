project(sparkfun_dmp_12e)

include(${ARDUINO_CMAKE_DIR}/esp8266/12e.cmake)

set(EXT_SRC_DIR 
                 ${SPARKFUN_DMP_LIB_SRC}
                 ${SPI_ESP8266_DIR}
                 ${WIRE_ESP8266_DIR}
                 ${ADAFRUIT_GFX_SRC}
                 ${PCD8544_ESP8266_SRC}
                 ${WIFI_ESP8266_DIR}
                 ${WEB_SOCKETS_DIR}
                 ${HASH_ESP8266_DIR}
                 ${ESP8266_ASYNC_UDP_DIR}
)

set(ext_sources 
    ${sparkfun_dmp_lib_sources}
    ${wire_sources}
    ${spi_sources}
    ${adafruit_esp8266_sources}
    ${wifi_sources}
    ${web_sockets_sources}
    ${HASH_ESP8266_DIR}/Hash.cpp
    ${ESP8266_ASYNC_UDP_DIR}/AsyncUDP.cpp
)

set(all_sources ${sources} ${ext_sources})
build_targets_esp8266_12e("${all_sources}" "${EXT_SRC_DIR}")
