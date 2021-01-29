cmake_minimum_required(VERSION 2.6)
project(camera_calibration)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS})

add_executable(camera_calibration main.cpp)

target_link_libraries(camera_calibration ${OpenCV_LIBS})


install(TARGETS camera_calibration RUNTIME DESTINATION bin)
