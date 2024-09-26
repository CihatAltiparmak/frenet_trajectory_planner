#include <nav_msgs/msg/path.hpp>

#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>

namespace frenet_trajectory_planner
{

class FrenetTrajectoryPlanner
{
public:
  FrenetTrajectoryPlanner();
  void plan();

private:
};

FrenetTrajectoryPlanner::FrenetTrajectoryPlanner()
{

}

void FrenetTrajectoryPlanner::plan(
  const CartesianState & robot_cartesian_state,
  const CartesianPoint & start_point,
  const CartesianPoint & final_point)
{

  auto frenet_trajectory_generator = FrenetTrajectoryGenerator(start_point, final_point);
  double min_lateral_distance = -1;
  double max_lateral_distance = 1;
  double step_lateral_distance = 0.5;
  double min_longtitutal_velocity = 0;
  double max_longtitutal_velocity = 2;
  double step_longtitutal_velocity = 0.5;
  // TODO (CihatAltiparmak) : eliminate some trajectories in frenet level
  auto all_frenet_trajectories = frenet_trajectory_generator.get_all_possible_frenet_trajectories(
    min_longtitutal_velocity, max_longtitutal_velocity, step_longtitutal_velocity,
    min_lateral_distance, max_lateral_distance, step_lateral_distance);
}

}
