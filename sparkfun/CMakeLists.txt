
set(sources ${CMAKE_CURRENT_SOURCE_DIR}/sparkfun.cpp
)

set(SPARKFUN_LIB_SRC ${PROJECT_ROOT}/SparkFun_MPU-9250_Breakout_Arduino_Library/src)
set(sparkfun_lib_sources ${SPARKFUN_LIB_SRC}/MPU9250.cpp
                         ${SPARKFUN_LIB_SRC}/quaternionFilters.cpp)

add_subdirectory(targets/atmega328)
