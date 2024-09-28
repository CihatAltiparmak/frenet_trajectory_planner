#include <vector>
#include <tuple>
#include <gtest/gtest.h>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>

#define _USE_MATH_DEFINES

TEST(frenet_trajectory_planner, conversion_adapters_line_adapter_test_initialization) {
  using frenet_trajectory_planner::CartesianPoint;

  CartesianPoint x_start;
  x_start << 1, 1;
  CartesianPoint x_finish;
  x_finish << 1, 2;
  auto adapter = frenet_trajectory_planner::LineAdapter(x_start, x_finish);
}

TEST(frenet_trajectory_planner, conversion_adapters_line_adapter_test_convert_frenet2cartesian) {
  using frenet_trajectory_planner::CartesianPoint;
  {

    CartesianPoint x_start;
    x_start << 1, 1;
    CartesianPoint x_finish;
    x_finish << 1, 2;
    auto adapter = frenet_trajectory_planner::LineAdapter(x_start, x_finish);

    frenet_trajectory_planner::FrenetState frenet_state =
      frenet_trajectory_planner::FrenetState::Zero();
    frenet_trajectory_planner::CartesianState cartesian_state = adapter.convert_frenet2cartesian(
      frenet_state);

    ASSERT_NEAR(cartesian_state[6], M_PI / 2, 1e-10);
  }

  {
    CartesianPoint x_start;
    x_start << 1, 1;
    CartesianPoint x_finish;
    x_finish << 1 + 1 / std::sqrt(2), 1 + 1 / std::sqrt(2);
    auto adapter = frenet_trajectory_planner::LineAdapter(x_start, x_finish);

    frenet_trajectory_planner::FrenetState frenet_state =
      frenet_trajectory_planner::FrenetState::Zero();
    frenet_trajectory_planner::CartesianState cartesian_state = adapter.convert_frenet2cartesian(
      frenet_state);

    ASSERT_NEAR(cartesian_state[6], M_PI / 4, 1e-10);
  }

  {
    CartesianPoint x_start;
    x_start << 1, 1;
    CartesianPoint x_finish;
    x_finish << 2, 1;
    auto adapter = frenet_trajectory_planner::LineAdapter(x_start, x_finish);

    frenet_trajectory_planner::FrenetState frenet_state =
      frenet_trajectory_planner::FrenetState::Zero();
    frenet_trajectory_planner::CartesianState cartesian_state = adapter.convert_frenet2cartesian(
      frenet_state);

    ASSERT_NEAR(cartesian_state[6], 0, 1e-10);
  }
}

TEST(frenet_trajectory_planner, conversion_adapters_line_adapter_test_convert_cartesian2frenet) {
  using frenet_trajectory_planner::CartesianPoint;
  {
    CartesianPoint x_start;
    x_start << 1, 1;
    CartesianPoint x_finish;
    x_finish << 1, 2;
    auto adapter = frenet_trajectory_planner::LineAdapter(x_start, x_finish);

    frenet_trajectory_planner::CartesianState cartesian_state;
    cartesian_state << -1, 0, 0, 1, 0, 0, M_PI / 2;

    frenet_trajectory_planner::FrenetState frenet_state = adapter.convert_cartesian2frenet(
      cartesian_state);

    ASSERT_NEAR(frenet_state[3], -2, 1e-10);
  }

  {
    CartesianPoint x_start;
    x_start << 1, 1;
    CartesianPoint x_finish;
    x_finish << 1, 2;
    auto adapter = frenet_trajectory_planner::LineAdapter(x_start, x_finish);

    frenet_trajectory_planner::CartesianState cartesian_state;
    cartesian_state << -1, 0, 0, 2, 0, 0, M_PI / 2;

    frenet_trajectory_planner::FrenetState frenet_state = adapter.convert_cartesian2frenet(
      cartesian_state);

    ASSERT_NEAR(frenet_state[3], -2, 1e-10);
    ASSERT_NEAR(frenet_state[0], 1, 1e-10);
  }

  {
    CartesianPoint x_start;
    x_start << 1, 1;
    CartesianPoint x_finish;
    x_finish << 1 + 1 / std::sqrt(2), 1 + 1 / std::sqrt(2);
    auto adapter = frenet_trajectory_planner::LineAdapter(x_start, x_finish);

    frenet_trajectory_planner::CartesianState cartesian_state;
    cartesian_state << -1, 0, 0, 3, 0, 0, M_PI / 2;

    frenet_trajectory_planner::FrenetState frenet_state = adapter.convert_cartesian2frenet(
      cartesian_state);

    ASSERT_NEAR(frenet_state[3], -2 * std::sqrt(2), 1e-10);
    ASSERT_NEAR(frenet_state[0], 0, 1e-10);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
