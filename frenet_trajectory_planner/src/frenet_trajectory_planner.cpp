#include <frenet_trajectory_planner/frenet_trajectory_planner.hpp>

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_selector.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>
#include <frenet_trajectory_planner/costs/lateral_distance_cost.hpp>
#include <frenet_trajectory_planner/costs/longtitutal_velocity_cost.hpp>

#include <memory>

namespace frenet_trajectory_planner
{

FrenetTrajectoryPlanner::FrenetTrajectoryPlanner()
{
  min_lateral_distance_ = -1;
  max_lateral_distance_ = 1;
  step_lateral_distance_ = 0.5;
  min_longtitutal_velocity_ = 0;
  max_longtitutal_velocity_ = 2;
  step_longtitutal_velocity_ = 0.5;
}

CartesianTrajectory FrenetTrajectoryPlanner::plan(
  const CartesianState & robot_cartesian_state,
  const CartesianPoint & start_point,
  const CartesianPoint & final_point)
{
  auto frenet2cartesian_converter =
    Frenet2CartesianConverter<LineAdapter>(start_point, final_point);
  auto cartesian2frenet_converter =
    Cartesian2FrenetConverter<LineAdapter>(start_point, final_point);

  auto robot_frenet_state = cartesian2frenet_converter.convert_state(robot_cartesian_state);

  auto frenet_trajectory_generator = FrenetTrajectoryGenerator(
    min_lateral_distance_,
    max_lateral_distance_,
    step_lateral_distance_,
    min_longtitutal_velocity_,
    max_longtitutal_velocity_,
    step_longtitutal_velocity_);
  // TODO (CihatAltiparmak) : eliminate some trajectories in frenet level
  auto all_frenet_trajectories = frenet_trajectory_generator.get_all_possible_frenet_trajectories(
    robot_frenet_state);

  auto frenet_trajectory_selector = FrenetTrajectorySelector();

  {
    auto lateral_distance_checker =
      std::make_shared<costs::LateralDistanceCost>(10);
    frenet_trajectory_selector.add_cost(lateral_distance_checker);
  }

  {
    auto longtitutal_velocity_cost_checker =
      std::make_shared<costs::LongtitutalVelocityCost>(10, 2);
    frenet_trajectory_selector.add_cost(longtitutal_velocity_cost_checker);
  }

  auto best_frenet_trajectory = frenet_trajectory_selector.select_best_frenet_trajectory(
    all_frenet_trajectories).value();

  auto best_cartesian_trajectory = frenet2cartesian_converter.convert_trajectory(
    best_frenet_trajectory);

  return best_cartesian_trajectory;
}

}
