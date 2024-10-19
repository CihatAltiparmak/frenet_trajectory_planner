#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>

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

  frenet_trajectory_planner::FrenetTrajectoryPlannerConfig planner_config;
  planner_config.min_lateral_distance = -1;
  planner_config.max_lateral_distance = 1;
  planner_config.step_lateral_distance = 0.5;
  planner_config.min_longtitutal_velocity = 0;
  planner_config.max_longtitutal_velocity = 2;
  planner_config.step_longtitutal_velocity = 0.5;

  auto frenet_trajectory_generator = frenet_trajectory_planner::FrenetTrajectoryGenerator(
    planner_config);
  // TODO (CihatAltiparmak) : eliminate some trajectories in frenet level
  auto all_frenet_trajectories = frenet_trajectory_generator.get_all_possible_frenet_trajectories(
    frenet_state_initial);

  auto frenet2cartesian_converter =
    frenet_trajectory_planner::Frenet2CartesianConverter<frenet_trajectory_planner::LineAdapter>(
    start_point, final_point);
  auto all_cartesian_trajectories = frenet2cartesian_converter.convert_trajectory_list(
    all_frenet_trajectories);

  return 0;
}
