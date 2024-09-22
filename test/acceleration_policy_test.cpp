#include <vector>
#include <tuple>
#include <gtest/gtest.h>
#include <frenet_trajectory_planner/policies/acceleration_policy.hpp>

TEST(frenet_trajectory_planner, acceleration_policy_test_initialization) {
  using FrenetTrajectoryArray = std::vector<std::tuple<double, double, double>>;
  using frenet_trajectory_planner::policies::AccelerationPolicy;
  using frenet_trajectory_planner::policies::AccelerationPolicyParameters;


  AccelerationPolicy<FrenetTrajectoryArray> acceleration_policy(AccelerationPolicyParameters{});
  ASSERT_EQ(4, 2 + 2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
