
set(MY_SRC ${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${ARDUINO_CORE_SRC} ${PROJECT_ROOT} ${MY_SRC})

set(sources ${MY_SRC}/Loop.cpp
            ${MY_SRC}/Setup.cpp
            ${MY_SRC}/Components.cpp
            ${MY_SRC}/Quaternion.cpp
            ${MY_SRC}/WireReader.cpp
)

add_subdirectory(atmega328)
add_subdirectory(ut)