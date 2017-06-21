cmake_minimum_required(VERSION 2.8)

project(mask_fusion)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

if(NOT APPLE)
  find_package(OpenMP)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

include_directories(/usr/include/eigen3)

include_directories(include)

add_executable(mask_view src/mask_view.cpp)
target_link_libraries(mask_view ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(mask_fusion src/mask_fusion.cpp)
target_link_libraries(mask_fusion ${OpenCV_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(label_view src/label_view.cpp)
target_link_libraries(label_view ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(label_fusion src/label_fusion.cpp)
target_link_libraries(label_fusion ${OpenCV_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(marching_cubes src/marching_cubes.cpp)
target_link_libraries(marching_cubes ${OpenCV_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(bilateral_filter src/bilateral_filter.cpp)
target_link_libraries(bilateral_filter ${OpenCV_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(organized_point_cloud src/organized_point_cloud.cpp)
target_link_libraries(organized_point_cloud ${OpenCV_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})
