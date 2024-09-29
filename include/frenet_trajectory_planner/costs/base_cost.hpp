#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>

namespace frenet_trajectory_planner
{
namespace costs
{

class Cost
{
public:
  Cost() {}
  virtual double cost(const FrenetTrajectory & frenet_trajectory) = 0;
};

}
}
