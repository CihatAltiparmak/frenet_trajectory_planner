#include <frenet_trajectory_planner/policies/base_policy.hpp>
#include <vector>

namespace frenet_trajectory_planner
{

class FrenetTrajectorySelector
{
public:
  FrenetTrajectorySelector();

  void add_policy(const policies::Policy & policy);

private:
  std::vector<policies::Policy> policies_;
};

FrenetTrajectorySelector::FrenetTrajectorySelector() {}

void FrenetTrajectorySelector::add_policy(const policies::Policy & policy)
{
  policies_.push_back(policy);
}

}
