cmake_minimum_required(VERSION 3.15)

project(Controller)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -Os -pthread -g")

find_package(yaml-cpp REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(matplotlib_cpp REQUIRED)

add_executable(Controller main.cpp
            ${PROJECT_SOURCE_DIR}/canport/can.cpp
            ${PROJECT_SOURCE_DIR}/canport/can.h
            ${PROJECT_SOURCE_DIR}/canport/can.hpp
            ${PROJECT_SOURCE_DIR}/motorsolver/motorsolver.cpp
            ${PROJECT_SOURCE_DIR}/motorsolver/motorsolver.hpp
)

target_link_libraries(Controller yaml-cpp matplotlib_cpp::matplotlib_cpp)