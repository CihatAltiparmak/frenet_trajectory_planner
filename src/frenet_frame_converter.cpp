namespace frenet_trajectory_planner
{

template<typename ConversionAdapter>
template<typename ... ConversionAdapterArgs>
class Frenet2CartesianConverter
{
public:
  Frenet2CartesianConverter(ConversionAdapterArgs... conversion_adapter_args);
  CartesianTrajectory convert_trajectory(const FrenetTrajectory & frenet_trajectory);
};

template<typename ConversionAdapter>
template<typename ... ConversionAdapterArgs>
class Cartesian2FrenetConverter
{
public:
  Cartesian2FrenetConverter(ConversionAdapterArgs... conversion_adapter_args);
  FrenetTrajectory convert_trajectory(const CartesianTrajectory & frenet_trajectory);
};

}
