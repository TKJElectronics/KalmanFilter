cmake_minimum_required(VERSION 3.19)
project(KalmanFilter)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories(examples)
include_directories(examples/MPU6050)

add_executable(KalmanFilter
        examples/MPU6050/I2C.ino
        examples/MPU6050/MPU6050.ino
        Kalman.cpp
        Kalman.h)
