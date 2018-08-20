cmake_minimum_required(VERSION 2.8)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

project(ceres-hello)
add_executable(${PROJECT_NAME} "main.cpp")
#add_executable(curve "curve_fitting.cpp")
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
#target_link_libraries(curve ${CERES_LIBRARIES})


