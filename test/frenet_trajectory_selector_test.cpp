#include <vector>
#include <tuple>
#include <gtest/gtest.h>
#include <frenet_trajectory_planner/frenet_trajectory_selector.hpp>
#include <frenet_trajectory_planner/policies/acceleration_policy.hpp>
#include <frenet_trajectory_planner/policies/base_policy.hpp>
#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>
#include <frenet_trajectory_planner/costs/distance_cost.hpp>

TEST(frenet_trajectory_planner, frenet_trajectory_selector_test_initialization) {
  using frenet_trajectory_planner::policies::AccelerationPolicy;
  using frenet_trajectory_planner::policies::AccelerationPolicyParameters;
  using frenet_trajectory_planner::CartesianTrajectory;
  using frenet_trajectory_planner::CartesianState;
  using frenet_trajectory_planner::LineAdapter;
  using frenet_trajectory_planner::CartesianPoint;

  CartesianPoint start_point;
  start_point << 0, 0;
  CartesianPoint final_point;
  final_point << 1, 0;
  auto acceleration_policy = std::make_shared<AccelerationPolicy<LineAdapter>>(
    AccelerationPolicyParameters{}, start_point, final_point);

  frenet_trajectory_planner::FrenetTrajectorySelector frenet_trajectory_selector;
  frenet_trajectory_selector.add_policy(acceleration_policy);

  auto lateral_distance_cost =
    std::make_shared<frenet_trajectory_planner::costs::LateralDistanceCost>(10);
  frenet_trajectory_selector.add_cost(lateral_distance_cost);
}

// TEST(frenet_trajectory_planner, frenet_trajectory_selector_test_get_feasible_cartesian_trajectories) {
//   using frenet_trajectory_planner::policies::AccelerationPolicy;
//   using frenet_trajectory_planner::policies::AccelerationPolicyParameters;
//   using frenet_trajectory_planner::CartesianTrajectory;
//   using frenet_trajectory_planner::CartesianState;
//   using frenet_trajectory_planner::LineAdapter;
//   using frenet_trajectory_planner::CartesianPoint;

//   CartesianPoint start_point;
//   start_point << 0, 0;
//   CartesianPoint final_point;
//   final_point << 1, 0;

//   AccelerationPolicyParameters parameters = {
//     -1.0, // acceleration_min
//     1.0   // acceleration_max
//   };
//   auto acceleration_policy = std::make_shared<AccelerationPolicy<LineAdapter>>(AccelerationPolicyParameters{}, start_point, final_point);

//   frenet_trajectory_planner::FrenetTrajectorySelector frenet_trajectory_selector;
//   frenet_trajectory_selector.add_policy(acceleration_policy);

//   std::vector<CartesianTrajectory> test_trajectory_array;

//   {
//     CartesianTrajectory trajectory;
//     CartesianState state1;
//     state1 << 0, 0, 0.5, 0, 0, 0, 0;
//     trajectory.push_back(state1);

//     test_trajectory_array.push_back(trajectory);
//   }

//   {
//     CartesianTrajectory trajectory;
//     CartesianState state1;
//     state1 << 0, 0, 1.5, 0, 0, 0.7, 0;
//     trajectory.push_back(state1);

//     test_trajectory_array.push_back(trajectory);
//   }
//   auto result_trajectory_array = frenet_trajectory_selector.get_feasible_cartesian_trajectories(
//     test_trajectory_array);
//   ASSERT_EQ(result_trajectory_array.size(), 1);
// }

// TEST(frenet_trajectory_planner, frenet_trajectory_selector_test_select_best_trajectory) {
//   using frenet_trajectory_planner::policies::AccelerationPolicy;
//   using frenet_trajectory_planner::policies::AccelerationPolicyParameters;
//   using frenet_trajectory_planner::CartesianTrajectory;
//   using frenet_trajectory_planner::CartesianState;
//   using frenet_trajectory_planner::LineAdapter;
//   using frenet_trajectory_planner::CartesianPoint;

//   CartesianPoint start_point;
//   start_point << 0, 0;
//   CartesianPoint final_point;
//   final_point << 1, 0;

//   AccelerationPolicyParameters parameters = {
//     -1.0, // acceleration_min
//     1.0   // acceleration_max
//   };
//   auto acceleration_policy = std::make_shared<AccelerationPolicy<LineAdapter>>(AccelerationPolicyParameters{}, start_point, final_point);

//   frenet_trajectory_planner::FrenetTrajectorySelector frenet_trajectory_selector;
//   frenet_trajectory_selector.add_policy(acceleration_policy);

//   auto lateral_distance_cost =
//     std::make_shared<frenet_trajectory_planner::costs::LateralDistanceCost>(10);
//   frenet_trajectory_selector.add_cost(lateral_distance_cost);

//   std::vector<CartesianTrajectory> test_trajectory_array;

//   {
//     CartesianTrajectory trajectory;
//     CartesianState state1;
//     state1 << 0, 0, 0.5, 0, 0, 0, 0;
//     trajectory.push_back(state1);

//     test_trajectory_array.push_back(trajectory);
//   }

//   {
//     CartesianTrajectory trajectory;
//     CartesianState state1;
//     state1 << 0, 0, 1.5, 0, 0, 0.7, 0;
//     trajectory.push_back(state1);

//     test_trajectory_array.push_back(trajectory);
//   }
//   auto result_trajectory_array = frenet_trajectory_selector.get_feasible_cartesian_trajectories(
//     test_trajectory_array);
//   ASSERT_EQ(result_trajectory_array.size(), 1);
// }

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
