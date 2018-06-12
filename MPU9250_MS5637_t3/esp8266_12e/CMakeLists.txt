
project(MPU9250_MS5637_t3_esp8266_12e)

include(${ARDUINO_CMAKE_DIR}/esp8266/paths.cmake)
include(${ARDUINO_CMAKE_DIR}/esp8266/tools.cmake)
include(${ARDUINO_CMAKE_DIR}/esp8266/defs.cmake)
include(${ARDUINO_CMAKE_DIR}/esp8266/deploy.cmake)

add_executable(${PROJECT_NAME} ${sources} 
            ${LCD_SRC}/LiquidCrystal_I2C.cpp
            ${WIRE_ESP8266_DIR}/Wire.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE ${esp8266_12e_linker_options})
add_dependencies(${PROJECT_NAME} ${esp8266_12e_core_libs})
target_compile_options(
    ${PROJECT_NAME} PRIVATE
    ${compiler_set_cpp}
    ${esp8266_12e_defs}
)
target_include_directories(
    ${PROJECT_NAME} PRIVATE
	${esp8266_12e_path}
    ${WIRE_ESP8266_DIR}
    ${SPI_ESP8266_DIR}
)
elf_to_bin_esp8266_12e(${PROJECT_NAME})
upload_esp8266_12e(${PROJECT_NAME} elf_to_bin_esp8266_12e_${PROJECT_NAME})

