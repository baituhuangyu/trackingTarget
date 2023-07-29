cmake_minimum_required(VERSION 3.10)
project(trackingTarget)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV 4.2)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 3.0)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 3.0 not found.")
        set(OpenCV_LIBS /root/repo/opencv/opencv_3416_ca53/lib/libopencv_world.so)
        set(OpenCV_INCLUDE_DIRS /root/repo/opencv/opencv_3416_ca53/include)
        set(OpenCV_VERSION 3.4.16)
    endif()
endif()
#set( CMAKE_C_COMPILER  /root/repo/toolchain/aarch64-ca53-linux-gnueabihf-8.4.01/bin/aarch64-ca53-linux-gnu-gcc )
#set( CMAKE_CXX_COMPILER  /root/repo/toolchain/aarch64-ca53-linux-gnueabihf-8.4.01/bin/aarch64-ca53-linux-gnu-g++ )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g -O0 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -flax-vector-conversions  -fpermissive ")
set(CMAKE_C_FLAGS "-g -O0")


MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

include_directories(.
        ${OpenCV_INCLUDE_DIRS}
        )

add_executable(trackingTarget
        fdssttracker.cpp
        fdssttracker.hpp
        ffttools.hpp
        fhog.cpp
        fhog.h
        labdata.hpp
        recttools.hpp
        sse.hpp
        sse2neon.h
        tracker.h
        trackingTarget.cpp
        )

target_link_libraries(trackingTarget
        ${OpenCV_LIBS}
        pthread
        )

