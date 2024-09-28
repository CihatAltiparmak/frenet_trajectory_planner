#ifndef _FRENET_TRAJECTORY_PLANNER__POLICIES__BASE_POLICY_
#define _FRENET_TRAJECTORY_PLANNER__POLICIES__BASE_POLICY_

#include <vector>
#include <iostream>

namespace frenet_trajectory_planner
{
namespace policies
{

class Policy {};

template<typename FrenetTrajectory, typename Parameters>
class BasePolicy : public Policy
{
public:
  BasePolicy(const Parameters & parameters)
  : parameters_(parameters)
  {
  }

  virtual std::vector<FrenetTrajectory> eliminate_trajectories(
    const std::vector<FrenetTrajectory> & frenet_trajectory_array)
  = 0;

  Parameters getParameters() const
  {
    return parameters_;
  }

protected:
  Parameters parameters_;
};

}
}

#endif
