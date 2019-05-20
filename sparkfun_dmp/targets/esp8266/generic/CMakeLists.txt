project(sparkfun_dmp_generic)

include(${ARDUINO_CMAKE_DIR}/esp8266/paths.cmake)
include(${ARDUINO_CMAKE_DIR}/esp8266/tools.cmake)
include(${ARDUINO_CMAKE_DIR}/esp8266/defs.cmake)
include(${ARDUINO_CMAKE_DIR}/esp8266/libs.cmake)
include(${ARDUINO_CMAKE_DIR}/esp8266/deploy.cmake)
include(${ARDUINO_CMAKE_DIR}/esp8266/env/wifi.cmake)

set(EXT_SRC_DIR 
                 ${SPARKFUN_DMP_LIB_SRC}
                 ${SPI_ESP8266_DIR}
                 ${WIRE_ESP8266_DIR}
                 ${ADAFRUIT_GFX_SRC}
                 ${PCD8544_ESP8266_SRC}
                 ${WIFI_ESP8266_DIR}
                 ${WEB_SOCKETS_DIR}
                 ${HASH_ESP8266_DIR}
)

set(ext_sources 
    ${sparkfun_dmp_lib_sources}
    ${wire_sources}
    ${spi_sources}
    ${adafruit_esp8266_sources}
    ${wifi_sources}
    ${web_sockets_sources}
    ${HASH_ESP8266_DIR}/Hash.cpp
)

add_executable(${PROJECT_NAME} ${sources} ${ext_sources})
target_link_libraries(${PROJECT_NAME} PRIVATE ${esp8266_linker_options})
add_dependencies(${PROJECT_NAME} ${esp8266_core_libs})
target_compile_options(
    ${PROJECT_NAME} PRIVATE
    ${compiler_set_cpp}
    ${esp8266_generic_defs}
    ${wifi_credentials}
)
target_include_directories(
    ${PROJECT_NAME} PRIVATE
    ${SRC_DIR}
    ${EXT_SRC_DIR}
    ${esp8266_generic_path}
)

elf_to_bin(${PROJECT_NAME})
upload(${PROJECT_NAME} elf_to_bin_${PROJECT_NAME})
