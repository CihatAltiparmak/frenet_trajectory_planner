#include <ilqr_trajectory_tracker/models/diff_robot_model.hpp>
#include <ilqr_trajectory_tracker/ilqr_optimizer.hpp>
#include <gtest/gtest.h>
#include <limits>

#define _USE_MATH_DEFINES

TEST(ilqr_trajectory_tracker, ilqr_optimizer_test_initialization) {

  using ilqr_trajectory_tracker::DiffDriveRobotModel;
  using ilqr_trajectory_tracker::DiffDriveRobotModelState;
  using ilqr_trajectory_tracker::DiffDriveRobotModelInput;

  DiffDriveRobotModel diff_drive_robot_model;

  ilqr_trajectory_tracker::NewtonOptimizer<DiffDriveRobotModel> newton_optimizer;

}

#include <iostream>
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
