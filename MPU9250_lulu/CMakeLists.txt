
set(MY_SRC ${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_ROOT} ${MY_SRC})

set(sources ${MY_SRC}/Mpu9250_lulu.cpp
)

add_subdirectory(targets/atmega328)
