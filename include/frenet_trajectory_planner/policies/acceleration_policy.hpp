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

template<typename FrenetTrajectoryArray>
class AccelerationPolicy : public BasePolicy<FrenetTrajectoryArray, AccelerationPolicyParameters>
{
public:
  AccelerationPolicy(const AccelerationPolicyParameters & acceleration_policy_parameters);
  void eliminate_trajectories(const FrenetTrajectoryArray & frenet_trajectory_array);
};

template class AccelerationPolicy<std::vector<std::tuple<double, double, double>>>;

}
}
