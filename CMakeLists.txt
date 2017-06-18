cmake_minimum_required(VERSION 2.8)

project(mask_fusion)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

find_package(PCL REQUIRED COMPONENTS io)
include_directories(${PCL_INCLUDE_DIRS})

include_directories(/usr/include/eigen3)
include_directories(include)

add_executable(mask_view src/mask_view.cpp)
target_link_libraries(mask_view ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(mask_fusion src/mask_fusion.cpp)
target_link_libraries(mask_fusion ${OpenCV_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(label_fusion src/label_fusion.cpp)
target_link_libraries(label_fusion ${OpenCV_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})
