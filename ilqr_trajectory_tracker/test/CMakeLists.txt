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
  diff_drive_robot_model_test
  models/diff_drive_robot_model_test.cpp)

target_link_libraries(diff_drive_robot_model_test  ilqr_trajectory_tracker_lib)

ament_add_gtest(
  ilqr_optimizer_test
  ilqr_optimizer_test.cpp)

target_link_libraries(ilqr_optimizer_test  ilqr_trajectory_tracker_lib)
