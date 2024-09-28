#include <vector>
#include <tuple>
#include <gtest/gtest.h>
#include <frenet_trajectory_planner/frenet_trajectory_selector.hpp>
#include <frenet_trajectory_planner/policies/acceleration_policy.hpp>
#include <frenet_trajectory_planner/policies/base_policy.hpp>

TEST(frenet_trajectory_planner, frenet_trajectory_selector_test_initialization) {
  struct FrenetState
  {
    double acceleration;
  };
  using frenet_trajectory_planner::policies::AccelerationPolicy;
  using frenet_trajectory_planner::policies::AccelerationPolicyParameters;
  using FrenetTrajectory = std::vector<FrenetState>;


  AccelerationPolicy<FrenetTrajectory> acceleration_policy(AccelerationPolicyParameters{});

  frenet_trajectory_planner::FrenetTrajectorySelector frenet_trajectory_selector;
  frenet_trajectory_selector.add_policy(acceleration_policy);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
