#include <vector>
#include <tuple>
#include <gtest/gtest.h>
#include <frenet_trajectory_planner/policies/acceleration_policy.hpp>


TEST(frenet_trajectory_planner, acceleration_policy_test_initialization) {


  using frenet_trajectory_planner::policies::State;
  using frenet_trajectory_planner::policies::AccelerationPolicy;
  using frenet_trajectory_planner::policies::AccelerationPolicyParameters;
  using FrenetTrajectory = std::vector<State>;


  AccelerationPolicy<FrenetTrajectory> acceleration_policy(AccelerationPolicyParameters{});
}

TEST(frenet_trajectory_planner, acceleration_policy_test_eliminate_trajectories) {
  using frenet_trajectory_planner::policies::State;
  using frenet_trajectory_planner::policies::AccelerationPolicy;
  using frenet_trajectory_planner::policies::AccelerationPolicyParameters;
  using FrenetTrajectory = std::vector<State>;

  AccelerationPolicyParameters parameters = {
    -1.0, // acceleration_min
    1.0   // acceleration_max
  };
  AccelerationPolicy<FrenetTrajectory> acceleration_policy(parameters);

  std::vector<FrenetTrajectory> test_trajectory_array;

  FrenetTrajectory trajectory1 = {{-0.5}, {0.7}};
  test_trajectory_array.push_back(trajectory1);

  FrenetTrajectory trajectory2 = {{-1.5}, {0.7}};
  test_trajectory_array.push_back(trajectory2);

  auto result_trajectory_array = acceleration_policy.eliminate_trajectories(test_trajectory_array);

  ASSERT_EQ(result_trajectory_array.size(), 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
