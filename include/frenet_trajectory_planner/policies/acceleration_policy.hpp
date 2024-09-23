#include <frenet_trajectory_planner/policies/base_policy.hpp>
#include <vector>
#include <tuple>

namespace frenet_trajectory_planner
{
namespace policies
{

typedef struct AccelerationLimits
{
  double acceleration_min;
  double acceleration_max;
} AccelerationPolicyParameters;

template<typename FrenetTrajectory>
class AccelerationPolicy : public BasePolicy<FrenetTrajectory, AccelerationPolicyParameters>
{
public:
  AccelerationPolicy(const AccelerationPolicyParameters & acceleration_policy_parameters);
  std::vector<FrenetTrajectory> eliminate_trajectories(
    const std::vector<FrenetTrajectory> & frenet_trajectory_array)
  override;
  bool check_trajectory_by_policy(const FrenetTrajectory & frenet_trajectory);
};

template<typename FrenetTrajectory>
AccelerationPolicy<FrenetTrajectory>::AccelerationPolicy(
  const AccelerationPolicyParameters & acceleration_policy_parameters)
: BasePolicy<FrenetTrajectory, AccelerationPolicyParameters>(acceleration_policy_parameters)
{

}

template<typename FrenetTrajectory>
std::vector<FrenetTrajectory> AccelerationPolicy<FrenetTrajectory>::eliminate_trajectories(
  const std::vector<FrenetTrajectory> & frenet_trajectory_array)
{
  std::vector<FrenetTrajectory> selected_frenet_trajectory_array;
  for (const auto & trajectory : frenet_trajectory_array) {
    if (check_trajectory_by_policy(trajectory)) {
      selected_frenet_trajectory_array.push_back(trajectory);
    }
  }

  return selected_frenet_trajectory_array;
}

template<typename FrenetTrajectory>
bool AccelerationPolicy<FrenetTrajectory>::check_trajectory_by_policy(
  const FrenetTrajectory & frenet_trajectory)
{
  for (const auto & state : frenet_trajectory) {
    if (state.acceleration > this->parameters_.acceleration_max ||
      state.acceleration < this->parameters_.acceleration_min)
    {
      return false;
    }
  }

  return true;
}

}
}
