#include <frenet_trajectory_planner/policies/base_policy.hpp>
#include <vector>

namespace frenet_trajectory_planner
{

class FrenetTrajectorySelector
{
public:
  FrenetTrajectorySelector();

private:
  std::vector<policies::BasePolicy> policies_;
};

}
