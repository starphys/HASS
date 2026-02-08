cmake_minimum_required(VERSION 3.10)

project(HASS)

file(GLOB_RECURSE SOURCE_FILES src/*.cpp)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(HASS ${SOURCE_FILES})