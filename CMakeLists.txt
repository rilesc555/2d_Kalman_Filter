cmake_minimum_required(VERSION 3.26)
project(2d_Kalman_Filter)

set(CMAKE_CXX_STANDARD 20)

add_executable(2d_Kalman_Filter KalmanFilter.h)

set_target_properties(2d_Kalman_Filter PROPERTIES
        LINKER_LANGUAGE CXX
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

target_include_directories(2d_Kalman_Filter PRIVATE ./Dependencies/eigen-3.4.0/)

