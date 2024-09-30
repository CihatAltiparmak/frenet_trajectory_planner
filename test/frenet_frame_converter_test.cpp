#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>

#include <gtest/gtest.h>


TEST(
  frenet_trajectory_planner,
  frenet_frame_converter_test_frenet2cartesian_converter_initialization) {
  using frenet_trajectory_planner::LineAdapter;
  using frenet_trajectory_planner::CartesianPoint;
  CartesianPoint start_point = CartesianPoint::Zero();
  CartesianPoint final_point;
  final_point << 0, 1;
  frenet_trajectory_planner::Frenet2CartesianConverter<LineAdapter> frenet2cartesian_converter(
    start_point, final_point);
}

TEST(
  frenet_trajectory_planner,
  frenet_frame_converter_test_frenet2cartesian_converter_convert_trajectory) {
  using frenet_trajectory_planner::LineAdapter;
  using frenet_trajectory_planner::CartesianPoint;
  using frenet_trajectory_planner::FrenetTrajectory;
  using frenet_trajectory_planner::CartesianTrajectory;
  using frenet_trajectory_planner::FrenetState;

  CartesianPoint start_point;
  start_point << 1, 1;
  CartesianPoint final_point;
  final_point << 1, 2;
  frenet_trajectory_planner::Frenet2CartesianConverter<LineAdapter> frenet2cartesian_converter(
    start_point, final_point);

  FrenetTrajectory frenet_trajectory;
  frenet_trajectory.push_back(FrenetState::Zero());

  CartesianTrajectory cartesian_trajectory = frenet2cartesian_converter.convert_trajectory(
    frenet_trajectory);

  ASSERT_NEAR(cartesian_trajectory[0][0], 1, 1e-10);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
