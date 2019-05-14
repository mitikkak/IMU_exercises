project(mpu6050_dmp6_generic)

include(${ARDUINO_CMAKE_DIR}/esp8266/generic.cmake)

set(EXT_SRC_DIR 
                 ${WIRE_ESP8266_DIR}
                 ${I2CDEV_DIR}
                 ${MPU6050_DIR}
)

set(ext_sources 
    ${mpu6050_sources}
    ${i2cdev_sources}
    ${wire_sources}
)

set(all_sources ${sources} ${ext_sources})
build_targets_esp8266_generic("${all_sources}" "${EXT_SRC_DIR}")
