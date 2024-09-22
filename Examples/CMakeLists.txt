CMAKE_MINIMUM_REQUIRED(VERSION 3.18.1)
project("ekCdt Examples")

file(GLOB CrossSrc
    ${CMAKE_CURRENT_SOURCE_DIR}/exCross.cpp)

add_executable(exCross ${CrossSrc})
