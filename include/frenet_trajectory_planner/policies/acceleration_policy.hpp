#include <frenet_trajectory_planner/policies/base_policy.hpp>
#include <frenet_trajectory_planner/frenet_frame_converter.hpp>
#include <vector>
#include <tuple>
#include <cmath>

namespace frenet_trajectory_planner
{
namespace policies
{

typedef struct AccelerationLimits
{
  double acceleration_min;
  double acceleration_max;
} AccelerationPolicyParameters;

template<typename ConversionAdapter>
class AccelerationPolicy : public BasePolicy<AccelerationPolicyParameters>
{
public:
  template<typename ... ConversionAdapterArgs>
  AccelerationPolicy(
    const AccelerationPolicyParameters & acceleration_policy_parameters,
    const ConversionAdapterArgs &... conversion_adapter_args);
  std::vector<FrenetTrajectory> eliminate_frenet_trajectories(
    const std::vector<FrenetTrajectory> & frenet_trajectory_array)
  override;
  bool check_frenet_trajectory_by_policy(const FrenetTrajectory & frenet_trajectory);

  std::vector<CartesianTrajectory> eliminate_cartesian_trajectories(
    const std::vector<CartesianTrajectory> & cartesian_trajectory_array) override;

  bool check_cartesian_trajectory_by_policy(
    const CartesianTrajectory & cartesian_trajectory);

private:
  Frenet2CartesianConverter<ConversionAdapter> frenet2cartesian_converter_;
};

template<typename ConversionAdapter>
template<typename ... ConversionAdapterArgs>
AccelerationPolicy<ConversionAdapter>::AccelerationPolicy(
  const AccelerationPolicyParameters & acceleration_policy_parameters,
  const ConversionAdapterArgs & ... conversion_adapter_args)
: BasePolicy<AccelerationPolicyParameters>(acceleration_policy_parameters),
  frenet2cartesian_converter_(conversion_adapter_args ...)
{

}

template<typename ConversionAdapter>
std::vector<FrenetTrajectory> AccelerationPolicy<ConversionAdapter>::eliminate_frenet_trajectories(
  const std::vector<FrenetTrajectory> & frenet_trajectory_array)
{
  std::vector<FrenetTrajectory> selected_frenet_trajectory_array;
  for (const auto & trajectory : frenet_trajectory_array) {
    if (check_frenet_trajectory_by_policy(trajectory)) {
      selected_frenet_trajectory_array.push_back(trajectory);
    }
  }

  return selected_frenet_trajectory_array;
}

template<typename ConversionAdapter>
bool AccelerationPolicy<ConversionAdapter>::check_frenet_trajectory_by_policy(
  const FrenetTrajectory & frenet_trajectory)
{
  CartesianTrajectory cartesian_trajectory = frenet2cartesian_converter_.convert_trajectory(
    frenet_trajectory);
  return this->check_cartesian_trajectory_by_policy(cartesian_trajectory);

  // for (const auto & state : frenet_trajectory) {
  //   double acceleration = (state({2, 5})).norm();
  //   if (acceleration > this->parameters_.acceleration_max ||
  //     acceleration < this->parameters_.acceleration_min)
  //   {
  //     return false;
  //   }
  // }

  // return true;
}

template<typename ConversionAdapter>
std::vector<CartesianTrajectory> AccelerationPolicy<ConversionAdapter>::
eliminate_cartesian_trajectories(
  const std::vector<CartesianTrajectory> & cartesian_trajectory_array)
{

  std::vector<CartesianTrajectory> selected_cartesian_trajectory_array;
  for (const auto & trajectory : cartesian_trajectory_array) {
    if (check_cartesian_trajectory_by_policy(trajectory)) {
      selected_cartesian_trajectory_array.push_back(trajectory);
    }
  }

  return selected_cartesian_trajectory_array;
}

template<typename ConversionAdapter>
bool AccelerationPolicy<ConversionAdapter>::check_cartesian_trajectory_by_policy(
  const CartesianTrajectory & cartesian_trajectory)
{
  for (const auto & state : cartesian_trajectory) {
    double acceleration = (state({2, 5})).norm();
    if (acceleration > this->parameters_.acceleration_max ||
      acceleration < this->parameters_.acceleration_min)
    {
      return false;
    }
  }

  return true;
}

}
}
