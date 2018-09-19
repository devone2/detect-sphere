cmake_minimum_required(VERSION 2.8)
project(detect-sphere)
set (CMAKE_CXX_STANDARD 11)
#pcl
find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


SET(TARGET_H "src/common.h")
SET(TARGET_SRC "src/common.cpp")

add_executable("detect-sphere" "src/cluster.cpp" ${TARGET_H} ${TARGET_SRC})
target_link_libraries("detect-sphere" ${PCL_LIBRARIES})



