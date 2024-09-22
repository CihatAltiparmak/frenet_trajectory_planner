#include <frenet_trajectory_planner/policies/base_policy.hpp>
#include <vector>
#include <tuple>

namespace frenet_trajectory_planner
{
namespace policies
{

typedef struct State
{
  double acceleration;
} State;

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

template class AccelerationPolicy<std::vector<State>>;

}
}
