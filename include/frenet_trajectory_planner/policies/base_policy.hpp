#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <vector>
#include <iostream>


namespace frenet_trajectory_planner
{
namespace policies
{

class Policy
{
public:
  Policy() {}
  virtual std::vector<FrenetTrajectory> eliminate_frenet_trajectories(
    const std::vector<FrenetTrajectory> & frenet_trajectory_array)
  = 0;

  virtual std::vector<CartesianTrajectory> eliminate_cartesian_trajectories(
    const std::vector<CartesianTrajectory> & cartesian_trajectory_array)
  = 0;

  virtual bool check_frenet_trajectory_by_policy(const FrenetTrajectory & frenet_trajectory) = 0;
  virtual bool check_cartesian_trajectory_by_policy(
    const CartesianTrajectory & cartesian_trajectory) = 0;
};

template<typename Parameters>
class BasePolicy : public Policy
{
public:
  BasePolicy(const Parameters & parameters)
  : parameters_(parameters)
  {
  }

  Parameters getParameters() const
  {
    return parameters_;
  }

protected:
  Parameters parameters_;
};

}
}
