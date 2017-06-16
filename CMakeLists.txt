cmake_minimum_required(VERSION 2.8)

project(mask_fusion)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(/usr/include/eigen3)

add_executable(mask_fusion src/mask_fusion.cpp)
target_link_libraries(mask_fusion ${OpenCV_LIBRARIES})
