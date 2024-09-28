cmake_minimum_required(VERSION 3.8)
project(frenet_trajectory_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  find_package(Eigen3 REQUIRED)
endif()

# if(NOT "${CMAKE_CXX_STANDARD}")
#     set(CMAKE_CXX_STANDARD 17)
#   endif()

find_package(ament_cmake REQUIRED)

add_subdirectory(demos)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
