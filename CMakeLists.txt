cmake_minimum_required(VERSION 3.26)
project(Radar_Kalman_Filter)

set(CMAKE_CXX_STANDARD 20)
add_subdirectory(Dependencies/csv-parser)


add_executable(Radar_Kalman_Filter KalmanFilter.h
        KalmanFilter.cpp
        main.cpp
)

set_target_properties(Radar_Kalman_Filter PROPERTIES
        LINKER_LANGUAGE CXX
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

include_directories(Radar_Kalman_Filter PRIVATE ./Dependencies/eigen-3.4.0/ ./Dependencies/csv-parser/)
target_link_libraries(Radar_Kalman_Filter csv)





