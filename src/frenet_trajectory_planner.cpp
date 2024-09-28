#include <frenet_trajectory_planner/frenet_trajectory_generator.hpp>
#include <frenet_trajectory_planner/conversion_adapters/line_adapter.hpp>

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
  (void) robot_cartesian_state;
  (void) start_point;
  (void) final_point;
}

}
