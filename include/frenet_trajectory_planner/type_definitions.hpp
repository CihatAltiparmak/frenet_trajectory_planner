#pragma once

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace frenet_trajectory_planner
{

using StateLateral = Vector3d;
using StateLongtitutal = Vector3d;
using FrenetState = Vector<double, 6>;
using FrenetTrajectory = std::vector<FrenetState>;

using CartesianState = Vector<double, 7>; // x, x_dot, x_dot_dot, y, y_dot, y_dot_dot, yaw
using CartesianTrajectory = std::vector<CartesianState>;

}
