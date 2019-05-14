
set(sources ${CMAKE_CURRENT_SOURCE_DIR}/mpu6050_dmp6.cpp
)
set(I2CDEV_DIR ${PROJECT_ROOT}/i2cdevlib/Arduino/I2Cdev)
set(MPU6050_DIR ${PROJECT_ROOT}/i2cdevlib/Arduino/MPU6050)

set(mpu6050_sources ${MPU6050_DIR}/MPU6050.cpp)
set(i2cdev_sources ${I2CDEV_DIR}/I2Cdev.cpp)
add_subdirectory(targets/esp8266/12e)
add_subdirectory(targets/esp8266/generic)
