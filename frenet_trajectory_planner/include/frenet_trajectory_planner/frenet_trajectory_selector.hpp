#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <frenet_trajectory_planner/policies/base_policy.hpp>
#include <frenet_trajectory_planner/costs/base_cost.hpp>
#include <vector>
#include <memory>
#include <optional>
#include <limits>

namespace frenet_trajectory_planner
{

class FrenetTrajectorySelector
{
public:
  FrenetTrajectorySelector();

  void add_policy(const std::shared_ptr<policies::Policy> & policy);
  void add_cost(const std::shared_ptr<costs::Cost> & cost);

  std::vector<FrenetTrajectory> get_feasible_frenet_trajectories(
    const std::vector<FrenetTrajectory> & frenet_trajectory);

  std::optional<FrenetTrajectory> select_best_frenet_trajectory(
    const std::vector<FrenetTrajectory> & frenet_trajectory);

private:
  std::vector<std::shared_ptr<policies::Policy>> policies_;
  std::vector<std::shared_ptr<costs::Cost>> costs_;
};

FrenetTrajectorySelector::FrenetTrajectorySelector() {}

void FrenetTrajectorySelector::add_policy(const std::shared_ptr<policies::Policy> & policy)
{
  policies_.push_back(policy);
}

void FrenetTrajectorySelector::add_cost(const std::shared_ptr<costs::Cost> & cost)
{
  costs_.push_back(cost);
}

// std::vector<CartesianTrajectory> FrenetTrajectorySelector::get_feasible_cartesian_trajectories(
//   const std::vector<CartesianTrajectory> & cartesian_trajectory)
// {

//   auto feasible_trajectories = cartesian_trajectory;
//   for (const auto & policy : policies_) {
//     feasible_trajectories = policy->eliminate_cartesian_trajectories(feasible_trajectories);
//   }

//   return feasible_trajectories;
// }

// template <typename ConversionAdapter>
// std::optional<CartesianTrajectory> FrenetTrajectorySelector::select_best_cartesian_trajectory(
//   const std::vector<CartesianTrajectory> & cartesian_trajectory)
// {

//   std::optional<CartesianTrajectory> best_cartesian_trajectory = std::nullopt;

//   for (const auto & frenet_trajectory : )

//   auto feasible_trajectories = cartesian_trajectory;


//   return best_cartesian_trajectory;
// }

std::vector<FrenetTrajectory> FrenetTrajectorySelector::get_feasible_frenet_trajectories(
  const std::vector<FrenetTrajectory> & frenet_trajectories)
{

  auto feasible_trajectories = frenet_trajectories;
  for (const auto & policy : policies_) {
    feasible_trajectories = policy->eliminate_frenet_trajectories(feasible_trajectories);
  }

  return feasible_trajectories;
}

std::optional<FrenetTrajectory> FrenetTrajectorySelector::select_best_frenet_trajectory(
  const std::vector<FrenetTrajectory> & frenet_trajectories)
{

  auto policy_checker = [this](const FrenetTrajectory & frenet_trajectory) -> bool {
      for (const auto & policy : policies_) {
        if (!policy->check_frenet_trajectory_by_policy(frenet_trajectory)) {
          return false;
        }
      }
      return true;
    };

  std::optional<FrenetTrajectory> best_frenet_trajectory = std::nullopt;
  double best_cost = std::numeric_limits<double>::infinity();
  for (const auto & frenet_trajectory : frenet_trajectories) {
    if (!policy_checker(frenet_trajectory)) {
      continue;
    }

    // check for cost
    double trajectory_cost = 0;
    for (const auto & cost_checker : costs_) {
      trajectory_cost += cost_checker->cost(frenet_trajectory);
    }
    if (trajectory_cost < best_cost) {
      best_cost = trajectory_cost;
      best_frenet_trajectory = std::optional<FrenetTrajectory>{frenet_trajectory};
    }
  }

  return best_frenet_trajectory;
}

}
