cmake_minimum_required(VERSION 3.8)
project(frenet_trajectory_planner)

add_link_options("-fuse-ld=mold")

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    acceleration_policy_test
    test/acceleration_policy_test.cpp)

  target_include_directories(
    acceleration_policy_test
    PUBLIC $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  ament_add_gtest(
    quintic_trajectory_planner_test
    test/quintic_trajectory_planner_test.cpp
    src/quintic_trajectory_planner.cpp)

  target_include_directories(quintic_trajectory_planner_test
    PUBLIC $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  target_include_directories(quintic_trajectory_planner_test PUBLIC ${Eigen3_INCLUDE_DIRS})
  target_link_libraries(quintic_trajectory_planner_test  Eigen3::Eigen)

  ament_add_gtest(
    quartic_trajectory_planner_test
    test/quartic_trajectory_planner_test.cpp
    src/quartic_trajectory_planner.cpp)

  target_include_directories(quartic_trajectory_planner_test
    PUBLIC $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  target_include_directories(quartic_trajectory_planner_test PUBLIC ${Eigen3_INCLUDE_DIRS})
  target_link_libraries(quartic_trajectory_planner_test  Eigen3::Eigen)

  ament_add_gtest(
    frenet_trajectory_generator_test
    test/frenet_trajectory_generator_test.cpp
    src/frenet_trajectory_generator.cpp
    src/quartic_trajectory_planner.cpp
    src/quintic_trajectory_planner.cpp)

  target_include_directories(frenet_trajectory_generator_test
    PUBLIC $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  target_include_directories(frenet_trajectory_generator_test PUBLIC ${Eigen3_INCLUDE_DIRS})
  target_link_libraries(frenet_trajectory_generator_test  Eigen3::Eigen)

  # if(TARGET Eigen3::Eigen)
  #   target_link_libraries(quintic_trajectory_planner_test PUBLIC Eigen3::Eigen)
  # else()
  #   target_include_directories(quintic_trajectory_planner_test PUBLIC ${Eigen3_INCLUDE_DIRS})
  # endif()

endif()

ament_package()
