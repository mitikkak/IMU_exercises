
set(sources ${CMAKE_CURRENT_SOURCE_DIR}/accelerometer_tilt.cpp
)

set(SPARKFUN_DMP_LIB_SRC ${PROJECT_ROOT}/SparkFun_MPU-9250-DMP_Arduino_Library/src)
set(sparkfun_dmp_lib_sources ${SPARKFUN_DMP_LIB_SRC}/SparkFunMPU9250-DMP.cpp
                         ${SPARKFUN_DMP_LIB_SRC}/util/arduino_mpu9250_i2c.cpp
                         ${SPARKFUN_DMP_LIB_SRC}/util/arduino_mpu9250_clk.c
                         ${SPARKFUN_DMP_LIB_SRC}/util/arduino_mpu9250_log.cpp
                         ${SPARKFUN_DMP_LIB_SRC}/util/inv_mpu.c
                         ${SPARKFUN_DMP_LIB_SRC}/util/inv_mpu_dmp_motion_driver.c
)
set(ESP8266_ASYNC_UDP_DIR ${PROJECT_ROOT}/ESPAsyncUDP/src)

add_subdirectory(targets/wroom32)
