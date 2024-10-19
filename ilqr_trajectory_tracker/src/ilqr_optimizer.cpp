namespace ilqr_trajectory_tracker
{

class Optimizer
{
public:
  Optimizer() {}
  // virtual void forward_pass();
  // virtual void backward_pass();
  // virtual void optimize();
};

template<typename RobotModel>
class NewtonOptimizer : public Optimizer
{
public:
  NewtonOptimizer();
  void backward_pass();
  void forward_pass();
  void optimize();

private:
  RobotModel robot_model_;
};

template<typename RobotModel>
NewtonOptimizer<RobotModel>::NewtonOptimizer()
: Optimizer()
{
}

template<typename RobotModel>
void NewtonOptimizer<RobotModel>::backward_pass(
  const std::vector<VectorXd> & x_feasible,
  const std::vector<VectorXd> & u_feasible,
  const MatrixXd & Q, const MatrixXd & R,
  const double dt)
{

  auto trajectory_size = x_feasible.size();
  auto state_dimension = Q.rows();
  auto input_dimension = R.rows();

  MatrixXd P_tilda = MatrixXd::Identity(state_dimension + 1, state_dimension + 1);
  P_tilda.topLeftCorner(state_dimension, state_dimension) = Q;

  std::vector<MatrixXd> K_gain(trajectory_size);

  for (int i = 0; i < trajectory_size - 1; i++) {
    auto x_offset =
      robot_model_.apply_system_dynamics(x_feasible[i], u_feasible[i], dt) - x_feasible[i + 1];

    MatrixXd A_tilda = MatrixXd::Identity(state_dimension + 1, state_dimension + 1);
    auto A = robot_model_.get_state_matrix(x_feasible[i], u_feasible[i], dt);
    A_tilda.topLeftCorner(state_dimension, state_dimension) = A;
    A_tilda.topRightCorner(state_dimension, 1) = x_offset;

    MatrixXd B_tilda = MatrixXd::Zero(state_dimension + 1, input_dimension);
    auto B = robot_model_.get_control_matrix(x_feasible[i], u_feasible[i]);
    B_tilda.topLeftCorner(state_dimension, input_dimension) = B;


  }
}

}
