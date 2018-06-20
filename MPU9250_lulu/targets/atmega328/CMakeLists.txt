
project(mpu9250_lulu_atmega328)

include(${ARDUINO_CMAKE_DIR}/atmega328/paths.cmake)
include(${ARDUINO_CMAKE_DIR}/atmega328/tools.cmake)
include(${ARDUINO_CMAKE_DIR}/atmega328/defs.cmake)
include(${ARDUINO_CMAKE_DIR}/atmega328/libs.cmake)
include(${ARDUINO_CMAKE_DIR}/atmega328/core.cmake)
include(${ARDUINO_CMAKE_DIR}/atmega328/deploy.cmake)

add_executable(${PROJECT_NAME} ${sources} ${atmega328_core_sources} ${arduino_wire_sources} ${arduino_timerone_sources})
target_link_libraries(${PROJECT_NAME} ${atmega328_linker_options})
target_compile_options(
    ${PROJECT_NAME} PRIVATE
    ${atmega328_compiler_set_cpp}
)
target_include_directories(
    ${PROJECT_NAME} PRIVATE
    ${ATMEGA328_CORE_SRC}
    ${ARDUINO_PINS}
    ${MY_SRC}
    ${ARDUINO_WIRE_SRC}
    ${ARDUINO_TIMERONE_SRC}
)

upload_atmega328(${PROJECT_NAME} ${PROJECT_NAME})
