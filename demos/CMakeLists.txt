add_executable(
  generate_all_possible_trajectories_demo
  generate_all_possible_trajectories_demo.cpp
  ../src/quintic_trajectory_planner.cpp
  ../src/frenet_trajectory_generator.cpp
  ../src/quartic_trajectory_planner.cpp
  ../src/quintic_trajectory_planner.cpp)

target_include_directories(
  generate_all_possible_trajectories_demo
  PUBLIC $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)

target_include_directories(generate_all_possible_trajectories_demo PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(generate_all_possible_trajectories_demo  Eigen3::Eigen)

add_executable(
  select_best_trajectory_demo
  select_best_trajectory_demo.cpp
  ../src/quintic_trajectory_planner.cpp
  ../src/frenet_trajectory_generator.cpp
  ../src/quartic_trajectory_planner.cpp
  ../src/quintic_trajectory_planner.cpp)

target_include_directories(
  select_best_trajectory_demo
  PUBLIC $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)

target_include_directories(select_best_trajectory_demo PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(select_best_trajectory_demo  Eigen3::Eigen)

install(TARGETS generate_all_possible_trajectories_demo
        select_best_trajectory_demo
        DESTINATION lib/frenet_trajectory_planner)