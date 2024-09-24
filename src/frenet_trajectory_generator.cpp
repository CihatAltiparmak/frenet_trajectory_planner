#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>

namespace frenet_trajectory_planner
{

FrenetTrajectoryGenerator::FrenetTrajectoryGenerator(
  double min_lateral_distance,
  double max_lateral_distance,
  double step_lateral_distance,
  double min_longtitutal_velocity,
  double max_longtitutal_velocity,
  double step_longtitutal_velocity)
:   min_lateral_distance_(min_lateral_distance),
  max_lateral_distance_(max_lateral_distance),
  step_lateral_distance_(step_lateral_distance),
  min_longtitutal_velocity_(min_longtitutal_velocity),
  max_longtitutal_velocity_(max_longtitutal_velocity),
  step_longtitutal_velocity_(step_longtitutal_velocity)
{}

std::vector<std::vector<FrenetState>> FrenetTrajectoryGenerator::
get_all_possible_frenet_trajectories(
  const FrenetState & frenet_state_initial)
{

  std::vector<std::vector<FrenetState>> frenet_trajectories;
  for (double longtitutal_velocity_final = min_longtitutal_velocity_;
    longtitutal_velocity_final < max_longtitutal_velocity_;
    longtitutal_velocity_final += step_longtitutal_velocity_)
  {
    for (double lateral_distance_final = min_lateral_distance_;
      lateral_distance_final < max_lateral_distance_;
      lateral_distance_final += step_lateral_distance_)
    {
      StateLongtitutal state_longtitutal_final;
      state_longtitutal_final << 0, longtitutal_velocity_final, 0;

      StateLateral state_lateral_final;
      state_lateral_final << lateral_distance_final, 0, 0;

      FrenetState frenet_state_final = {state_longtitutal_final, state_lateral_final};
      auto frenet_trajectory = get_frenet_trajectory(frenet_state_initial, frenet_state_final);
      frenet_trajectories.push_back(frenet_trajectory);
    }
  }

  return frenet_trajectories;
}

std::vector<FrenetState> FrenetTrajectoryGenerator::get_frenet_trajectory(
  const FrenetState & frenet_state_initial,
  const FrenetState & frenet_state_final)
{
  auto longtitual_state_initial = std::get<0>(frenet_state_initial);
  auto longtitual_state_final = std::get<0>(frenet_state_final);
  auto longtitutal_velocity_planner = QuarticTrajectoryPlanner();
  if (longtitutal_velocity_planner.set_coefficients_or_return_false(
      longtitual_state_initial[0], longtitual_state_initial[1], longtitual_state_initial[2],
      longtitual_state_final[1], longtitual_state_final[2],
      0, 1))
  {
    return {};
  }

  auto lateral_state_initial = std::get<1>(frenet_state_initial);
  auto lateral_state_final = std::get<1>(frenet_state_final);
  auto lateral_distance_planner = QuinticTrajectoryPlanner();
  if (lateral_distance_planner.set_coefficients_or_return_false(
      lateral_state_initial[0], lateral_state_initial[1], lateral_state_initial[2],
      lateral_state_final[1], lateral_state_final[1], lateral_state_final[2],
      0, 1))
  {
    return {};
  }

  std::vector<FrenetState> frenet_trajectory;

  for (double t = 0; t < 1.0; t += 0.1) {
    StateLongtitutal state_longtitutal;
    state_longtitutal[0] = longtitutal_velocity_planner.x(t);
    state_longtitutal[1] = longtitutal_velocity_planner.dx(t);
    state_longtitutal[2] = longtitutal_velocity_planner.ddx(t);

    StateLateral state_lateral;
    state_lateral[0] = lateral_distance_planner.x(t);
    state_lateral[1] = lateral_distance_planner.dx(t);
    state_lateral[2] = lateral_distance_planner.ddx(t);

    frenet_trajectory.push_back({state_longtitutal, state_lateral});
  }

  return frenet_trajectory;
}

}
