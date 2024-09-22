#ifndef _FRENET_TRAJECTORY_PLANNER__POLICIES__BASE_POLICY_
#define _FRENET_TRAJECTORY_PLANNER__POLICIES__BASE_POLICY_

namespace frenet_trajectory_planner
{
namespace policies
{

template<typename FrenetTrajectoryArray, typename Parameters>
class BasePolicy
{
public:
  BasePolicy(const Parameters & parameters)
  : parameters_(parameters)
  {
  }

  virtual void eliminate_trajectories(const FrenetTrajectoryArray & frenet_trajectory_array);

private:
  Parameters parameters_;
};

}
}

#endif
