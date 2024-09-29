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
  acceleration_policy_test.cpp)

target_include_directories(
  acceleration_policy_test
  PUBLIC $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)

target_include_directories(acceleration_policy_test PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(acceleration_policy_test  Eigen3::Eigen)

ament_add_gtest(
  quintic_trajectory_planner_test
  quintic_trajectory_planner_test.cpp
  ../src/quintic_trajectory_planner.cpp)

target_include_directories(quintic_trajectory_planner_test
  PUBLIC $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)

target_include_directories(quintic_trajectory_planner_test PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(quintic_trajectory_planner_test  Eigen3::Eigen)

ament_add_gtest(
  quartic_trajectory_planner_test
  quartic_trajectory_planner_test.cpp
  ../src/quartic_trajectory_planner.cpp)

target_include_directories(quartic_trajectory_planner_test
  PUBLIC $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)

target_include_directories(quartic_trajectory_planner_test PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(quartic_trajectory_planner_test  Eigen3::Eigen)

ament_add_gtest(
  frenet_trajectory_generator_test
  frenet_trajectory_generator_test.cpp
  ../src/frenet_trajectory_generator.cpp
  ../src/quartic_trajectory_planner.cpp
  ../src/quintic_trajectory_planner.cpp)

target_include_directories(frenet_trajectory_generator_test
  PUBLIC $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)

target_include_directories(frenet_trajectory_generator_test PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(frenet_trajectory_generator_test  Eigen3::Eigen)

ament_add_gtest(
  line_adapter_test
  conversion_adapters/line_adapter_test.cpp)

target_include_directories(line_adapter_test
  PUBLIC $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)

target_include_directories(line_adapter_test PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(line_adapter_test  Eigen3::Eigen)

ament_add_gtest(
  frenet_frame_converter_test
  frenet_frame_converter_test.cpp)

target_include_directories(frenet_frame_converter_test
  PUBLIC $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)

target_include_directories(frenet_frame_converter_test PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(frenet_frame_converter_test  Eigen3::Eigen)

ament_add_gtest(
  frenet_trajectory_selector_test
  frenet_trajectory_selector_test.cpp)

target_include_directories(frenet_trajectory_selector_test
  PUBLIC $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)

target_include_directories(frenet_trajectory_selector_test PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(frenet_trajectory_selector_test  Eigen3::Eigen)

# if(TARGET Eigen3::Eigen)
#   target_link_libraries(quintic_trajectory_planner_test PUBLIC Eigen3::Eigen)
# else()
#   target_include_directories(quintic_trajectory_planner_test PUBLIC ${Eigen3_INCLUDE_DIRS})
# endif()