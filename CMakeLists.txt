cmake_minimum_required(VERSION 2.8)
project(ceres-hello)

# ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} "main.cpp")
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})

#pcl
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable("detect-sphere" "pclmain.cpp")
target_link_libraries("detect-sphere" ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})


#add_executable(curve "curve_fitting.cpp")
#target_link_libraries(curve ${CERES_LIBRARIES})


