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

  CartesianState convert_state(const FrenetState & frenet_state);
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
CartesianState Frenet2CartesianConverter<ConversionAdapter>::convert_state(
  const FrenetState & frenet_state)
{
  CartesianState cartesian_state = conversion_adapter_.convert_frenet2cartesian(frenet_state);

  return cartesian_state;
}

template<typename ConversionAdapter>
CartesianTrajectory Frenet2CartesianConverter<ConversionAdapter>::convert_trajectory(
  const FrenetTrajectory & frenet_trajectory)
{
  CartesianTrajectory cartesian_trajectory;
  for (const auto & frenet_state : frenet_trajectory) {
    auto cartesian_state = convert_state(frenet_state);
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

// ----------------------------------

template<typename ConversionAdapter>
class Cartesian2FrenetConverter
{
public:
  template<typename ... ConversionAdapterArgs>
  Cartesian2FrenetConverter(const ConversionAdapterArgs & ... conversion_adapter_args);

  FrenetState convert_state(const CartesianState & cartesian_state);
  FrenetTrajectory convert_trajectory(const CartesianTrajectory & cartesian_trajectory);
  std::vector<FrenetTrajectory> convert_trajectory_list(
    const std::vector<CartesianTrajectory> & cartesian_trajectory_list);

private:
  ConversionAdapter conversion_adapter_;
};

template<typename ConversionAdapter>
template<typename ... ConversionAdapterArgs>
Cartesian2FrenetConverter<ConversionAdapter>::Cartesian2FrenetConverter(
  const ConversionAdapterArgs & ... conversion_adapter_args)
: conversion_adapter_(conversion_adapter_args ...)
{

}

template<typename ConversionAdapter>
FrenetState Cartesian2FrenetConverter<ConversionAdapter>::convert_state(
  const CartesianState & cartesian_state)
{
  FrenetState frenet_state = conversion_adapter_.convert_cartesian2frenet(cartesian_state);

  return frenet_state;
}

template<typename ConversionAdapter>
FrenetTrajectory Cartesian2FrenetConverter<ConversionAdapter>::convert_trajectory(
  const CartesianTrajectory & cartesian_trajectory)
{
  FrenetTrajectory frenet_trajectory;
  for (const auto & cartesian_state : cartesian_trajectory) {
    auto frenet_state = convert_state(cartesian_state);
    frenet_trajectory.push_back(frenet_state);
  }

  return frenet_trajectory;
}

template<typename ConversionAdapter>
std::vector<FrenetTrajectory> Cartesian2FrenetConverter<ConversionAdapter>::convert_trajectory_list(
  const std::vector<CartesianTrajectory> & cartesian_trajectory_list)
{
  std::vector<FrenetTrajectory> frenet_trajectory_list;

  for (const auto & cartesian_trajectory : cartesian_trajectory_list) {
    FrenetTrajectory frenet_trajectory = convert_trajectory(cartesian_trajectory);
    frenet_trajectory_list.push_back(frenet_trajectory);
  }

  return frenet_trajectory_list;
}

}
