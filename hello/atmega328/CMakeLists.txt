
project(hello)

include(${PROJECT_ROOT}/arduino_cmake/arduino_paths.cmake)
set(AVR_COMPILER_DIR ${ARDUINO_PATH}/hardware/tools/avr/bin)
set(ARDUINO_CORE_SRC ${ARDUINO_PATH}/hardware/arduino/avr/cores/arduino)
set(ARDUINO_WIRE_SRC ${ARDUINO_PATH}/hardware/arduino/avr/libraries/Wire/src/)
set(ARDUINO_PINS ${ARDUINO_PATH}/hardware/arduino/avr/variants/standard)
set(ARDUINO_SPI_SRC ${ARDUINO_PATH}/hardware/arduino/avr/libraries/SPI/src)

include(${PROJECT_ROOT}/arduino_cmake/arduino_core.cmake)
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")   # remove -rdynamic for C
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "") # remove -rdynamic for CXX
include(${PROJECT_ROOT}/arduino_cmake/arduino_tools.cmake)
include(${PROJECT_ROOT}/arduino_cmake/atmega328_defs.cmake)
INCLUDE_DIRECTORIES(${ARDUINO_CORE_SRC} ${ARDUINO_PINS} ${ARDUINO_SPI_SRC} ${PROJECT_ROOT})
add_executable(${PROJECT_NAME} ${sources} ${arduino_core_sources} ${spi_sources})

add_custom_target(elf_to_hex_${PROJECT_NAME} COMMAND ${ARDUINO_PATH}/hardware/tools/avr/bin/avr-objcopy -O ihex -R .eeprom ${PROJECT_NAME} ${PROJECT_NAME}.hex)
add_custom_target(upload_${PROJECT_NAME} COMMAND ${ARDUINO_PATH}/hardware/tools/avr/bin/avrdude -C${ARDUINO_PATH}/hardware/tools/avr/etc/avrdude.conf -v -patmega328p -carduino -P/dev/ttyACM0 -b115200 -Uflash:w:./${PROJECT_NAME}.hex:i
                                    DEPENDS elf_to_hex_${PROJECT_NAME})
