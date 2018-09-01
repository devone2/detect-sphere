cmake_minimum_required(VERSION 2.8)
project(detect-sphere)

#pcl
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable("detect-sphere" "pclmain.cpp")
target_link_libraries("detect-sphere" ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})



