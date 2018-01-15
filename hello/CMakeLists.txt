
set(HELLO_SRC ${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${ARDUINO_CORE_SRC} ${PROJECT_ROOT})

set(sources ${HELLO_SRC}/Loop.cpp
            ${HELLO_SRC}/Setup.cpp
)

add_subdirectory(atmega328)