#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <frenet_trajectory_planner/frenet_trajectory_selector.hpp>
#include <frenet_trajectory_planner/policies/acceleration_policy.hpp>
#include <frenet_trajectory_planner/costs/distance_cost.hpp>

int main()
{

  frenet_trajectory_planner::CartesianState robot_cartesian_state;
  frenet_trajectory_planner::CartesianPoint start_point;
  start_point << 1, 1;
  frenet_trajectory_planner::CartesianPoint final_point;
  final_point << 5, 1;
  frenet_trajectory_planner::FrenetState frenet_state_initial =
    frenet_trajectory_planner::FrenetState::Zero();
  frenet_state_initial[0] = 0;
  frenet_state_initial[1] = 1;
  frenet_state_initial[3] = 0.7;
  frenet_state_initial[4] = 0.5;

  double min_lateral_distance = -1;
  double max_lateral_distance = 1;
  double step_lateral_distance = 0.5;
  double min_longtitutal_velocity = 0;
  double max_longtitutal_velocity = 2;
  double step_longtitutal_velocity = 0.5;

  auto frenet_trajectory_generator = frenet_trajectory_planner::FrenetTrajectoryGenerator(
    min_lateral_distance,
    max_lateral_distance,
    step_lateral_distance,
    min_longtitutal_velocity,
    max_longtitutal_velocity,
    step_longtitutal_velocity);
  // TODO (CihatAltiparmak) : eliminate some trajectories in frenet level
  auto all_frenet_trajectories = frenet_trajectory_generator.get_all_possible_frenet_trajectories(
    frenet_state_initial);

  auto frenet_trajectory_selector = frenet_trajectory_planner::FrenetTrajectorySelector();

  {
    auto lateral_distance_checker =
      std::make_shared<frenet_trajectory_planner::costs::LateralDistanceCost>(10);
    frenet_trajectory_selector.add_cost(lateral_distance_checker);
  }

  auto best_frenet_trajectory = frenet_trajectory_selector.select_best_frenet_trajectory(
    all_frenet_trajectories).value();
  auto frenet2cartesian_converter =
    frenet_trajectory_planner::Frenet2CartesianConverter<frenet_trajectory_planner::LineAdapter>(
    start_point, final_point);

  auto best_cartesian_trajectory = frenet2cartesian_converter.convert_trajectory(
    best_frenet_trajectory);

  return 0;
}
