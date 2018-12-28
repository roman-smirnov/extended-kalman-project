cmake_minimum_required(VERSION 3.12)
project(extended-kalman-project)

set(CMAKE_CXX_STANDARD 17)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(SOURCES src/main.cpp src/kalman/tools.cpp src/kalman/FusionEKF.cpp src/kalman/kalman_filter.cpp
        src/controller.cpp src/kalman/tools.h src/kalman/FusionEKF.h src/kalman/kalman_filter.h
        src/controller.h src/network.cpp src/network.h)

set(LIBRARIES z ssl uv uWS)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories(third-party/include)  # where to search for headers
    link_directories(third-party/lib)  # where to search for binaries
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(extended_kalman ${SOURCES})
target_link_libraries(extended_kalman ${LIBRARIES})








