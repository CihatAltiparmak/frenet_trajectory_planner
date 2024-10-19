#pragma once

#include <ilqr_trajectory_tracker/models/base_model.hpp>

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace ilqr_trajectory_tracker
{

using DiffDriveRobotModelState = Vector3d;
using DiffDriveRobotModelInput = Vector2d;

class DiffDriveRobotModel : public Model<DiffDriveRobotModelState, DiffDriveRobotModelInput>
{
public:
  using StateT = DiffDriveRobotModelState;
  using InputT = DiffDriveRobotModelInput;
  DiffDriveRobotModel();
  Vector3d apply_system_dynamics(const Vector3d & x, const Vector2d & u, const double dt) override;
  MatrixXd get_state_matrix(const Vector3d & x_eq, const Vector2d & u_eq, const double dt);
  MatrixXd get_control_matrix(const Vector3d & x_eq, const Vector2d & u_eq, const double dt);
};

}
