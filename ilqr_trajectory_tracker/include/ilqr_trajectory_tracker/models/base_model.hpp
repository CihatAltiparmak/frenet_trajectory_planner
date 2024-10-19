#pragma once

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

namespace ilqr_trajectory_tracker
{

template<typename StateT, typename InputT>
class Model
{
public:
  Model();
  virtual StateT apply_system_dynamics(const StateT & x, const InputT & u, const double dt) = 0;
  virtual MatrixXd get_state_matrix(const StateT & x_eq, const InputT & u_eq, const double dt) = 0;
  virtual MatrixXd get_control_matrix(
    const StateT & x_eq, const InputT & u_eq,
    const double dt) = 0;
};

template<typename StateT, typename InputT>
Model<StateT, InputT>::Model()
{
}

}
