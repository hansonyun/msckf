cmake_minimum_required(VERSION 3.3)
project(msckf)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

find_package (glog REQUIRED)

file(GLOB_RECURSE APP_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/app/*.cpp)
file(GLOB_RECURSE SIM_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/sim/*.cpp)
file(GLOB_RECURSE STATE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/state/*.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(msckf_self ${SIM_SRCS} ${STATE_SRCS})
target_link_libraries(msckf_self Eigen3::Eigen glog::glog)

add_executable(main ${APP_SRCS})
target_link_libraries(main msckf_self)


