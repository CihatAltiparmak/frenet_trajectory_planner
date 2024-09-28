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

  frenet_trajectory_planner::FrenetTrajectoryGenerator frenet_trajectory_generator(-1, 1, 0.1, -1,
    1,
    0.1);

  FrenetState frenet_state_initial = FrenetState::Zero();
  frenet_state_initial[1] = 1;
  frenet_state_initial[3] = -1;
  auto all_frenet_trajectories = frenet_trajectory_generator.get_all_possible_frenet_trajectories(
    frenet_state_initial);

  ASSERT_EQ(all_frenet_trajectories.size(), 441u);

  auto first_generated_frenet_trajectory = all_frenet_trajectories[0];
  auto final_state_of_first_trajectory = first_generated_frenet_trajectory.back();
  // check if the final longtitutal velocity is -1
  ASSERT_NEAR(final_state_of_first_trajectory[1], -1, 1e-10);
  // check if the lateral distance is 0
  ASSERT_NEAR(final_state_of_first_trajectory[3], -1, 1e-10);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
