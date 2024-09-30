#pragma once

#include <frenet_trajectory_planner/type_definitions.hpp>
#include <vector>

namespace frenet_trajectory_planner
{

template<typename ConversionAdapter>
class Frenet2CartesianConverter
{
public:
  template<typename ... ConversionAdapterArgs>
  Frenet2CartesianConverter(const ConversionAdapterArgs & ... conversion_adapter_args);

  CartesianTrajectory convert_trajectory(const FrenetTrajectory & frenet_trajectory);
  std::vector<CartesianTrajectory> convert_trajectory_list(
    const std::vector<FrenetTrajectory> & frenet_trajectory_list);

private:
  ConversionAdapter conversion_adapter_;
};

template<typename ConversionAdapter>
template<typename ... ConversionAdapterArgs>
Frenet2CartesianConverter<ConversionAdapter>::Frenet2CartesianConverter(
  const ConversionAdapterArgs & ... conversion_adapter_args)
: conversion_adapter_(conversion_adapter_args ...)
{

}

template<typename ConversionAdapter>
CartesianTrajectory Frenet2CartesianConverter<ConversionAdapter>::convert_trajectory(
  const FrenetTrajectory & frenet_trajectory)
{
  CartesianTrajectory cartesian_trajectory;
  for (const auto & frenet_state : frenet_trajectory) {
    auto cartesian_state = conversion_adapter_.convert_frenet2cartesian(frenet_state);
    cartesian_trajectory.push_back(cartesian_state);
  }

  return cartesian_trajectory;
}

template<typename ConversionAdapter>
std::vector<CartesianTrajectory> Frenet2CartesianConverter<ConversionAdapter>::
convert_trajectory_list(const std::vector<FrenetTrajectory> & frenet_trajectory_list)
{
  std::vector<CartesianTrajectory> cartesian_trajectory_list;

  for (const auto & frenet_trajectory : frenet_trajectory_list) {
    CartesianTrajectory cartesian_trajectory = convert_trajectory(frenet_trajectory);
    cartesian_trajectory_list.push_back(cartesian_trajectory);
  }

  return cartesian_trajectory_list;
}

// template<typename ConversionAdapter>
// template<typename ... ConversionAdapterArgs>
// class Cartesian2FrenetConverter
// {
// public:
//   Cartesian2FrenetConverter(ConversionAdapterArgs... conversion_adapter_args);
//   FrenetTrajectory convert_trajectory(const CartesianTrajectory & frenet_trajectory);
// };

}
