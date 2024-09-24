#include <vector>
#include <tuple>
#include <gtest/gtest.h>
#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>


TEST(frenet_trajectory_planner, frenet_trajectory_generator_test_initialization) {

  frenet_trajectory_planner::FrenetTrajectoryGenerator frenet_trajectory_generator(0, 1, 0.1, 0, 1,
    0.1);
}

TEST(
  frenet_trajectory_planner,
  frenet_trajectory_generator_test_get_all_possible_frenet_trajectories) {

  using frenet_trajectory_planner::FrenetState;

  frenet_trajectory_planner::FrenetTrajectoryGenerator frenet_trajectory_generator(0, 1, 0.1, 0, 1,
    0.1);

  FrenetState frenet_state_initial;
  frenet_trajectory_generator.get_all_possible_frenet_trajectories(frenet_state_initial);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
