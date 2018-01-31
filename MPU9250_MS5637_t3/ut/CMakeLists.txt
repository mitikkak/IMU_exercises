
project(unit_test_MPU9250_MS5637_t3)

INCLUDE_DIRECTORIES(${PROJECT_ROOT}/googletest/googletest/include/ ${PROJECT_ROOT}/arduino_cmake/ut/stubs/)

set(arduino_stubs  ${PROJECT_ROOT}/arduino_cmake/ut/stubs/Arduino.cpp 
                   ${PROJECT_ROOT}/arduino_cmake/ut/stubs/Wire.cpp
                   ${PROJECT_ROOT}/arduino_cmake/ut/stubs/LiquidCrystal.cpp
)
set(runner ${PROJECT_ROOT}/arduino_cmake/ut/Runner.cpp)
set(tests ${CMAKE_CURRENT_SOURCE_DIR}/TestMadgwickQuaternionUpdate.cpp
)
add_definitions(-c -g -w -std=gnu++11)
add_executable(${PROJECT_NAME} ${tests} ${sources} ${runner} ${arduino_stubs})
set(GTEST_LIBRARIES gtest gmock)
target_link_libraries(${PROJECT_NAME} ${GTEST_LIBRARIES} pthread)
