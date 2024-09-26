#include <tuple>
#include <vector>
#include <frenet_trajectory_planner/quartic_trajectory_planner.hpp>
#include <frenet_trajectory_planner/quintic_trajectory_planner.hpp>

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace frenet_trajectory_planner
{

using StateLateral = Vector3d;
using StateLongtitutal = Vector3d;
using FrenetState = Vector<double, 6>;
using FrenetTrajectory = std::vector<FrenetState>;

class FrenetTrajectoryGenerator
{
public:
  FrenetTrajectoryGenerator(
    double min_lateral_distance,
    double max_lateral_distance,
    double step_lateral_distance,
    double min_longtitutal_velocity,
    double max_longtitutal_velocity,
    double step_longtitutal_velocity);

  std::vector<FrenetTrajectory> get_all_possible_frenet_trajectories(
    const FrenetState & frenet_state_initial);

  FrenetTrajectory get_frenet_trajectory(
    const FrenetState & frenet_state_initial,
    const FrenetState & frenet_state_final);

private:
  double min_lateral_distance_;
  double max_lateral_distance_;
  double step_lateral_distance_;

  double min_longtitutal_velocity_;
  double max_longtitutal_velocity_;
  double step_longtitutal_velocity_;
};

}
