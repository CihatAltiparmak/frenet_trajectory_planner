#include <frenet_trajectory_planner/policies/acceleration_policy.hpp>

namespace frenet_trajectory_planner
{
namespace policies
{

template<typename FrenetTrajectoryArray>
AccelerationPolicy<FrenetTrajectoryArray>::AccelerationPolicy(
  const AccelerationPolicyParameters & acceleration_policy_parameters)
: BasePolicy<FrenetTrajectoryArray, AccelerationPolicyParameters>(acceleration_policy_parameters)
{

}

template<typename FrenetTrajectoryArray>
void AccelerationPolicy<FrenetTrajectoryArray>::eliminate_trajectories(
  const FrenetTrajectoryArray & /*frenet_trajectory_array*/)
{
}

}
}
