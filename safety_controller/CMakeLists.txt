cmake_minimum_required(VERSION 3.0)
project(safety_controller_project)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lrt")

# find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(safety_controller safety_controller.cpp)
# target_link_libraries (safety_controller Eigen3::Eigen)
target_link_libraries (safety_controller -lrt)

