project(without_gui_project)

cmake_minimum_required(VERSION 3.0)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(without_gui without_gui.cpp without_gui.h)
# target_link_libraries (without_gui Eigen3::Eigen)
target_link_libraries (without_gui -lrt)

