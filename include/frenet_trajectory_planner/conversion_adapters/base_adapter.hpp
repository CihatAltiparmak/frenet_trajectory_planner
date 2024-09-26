namespace frenet_trajectory_planner
{

template<typename ... AdapterArgs>
class BaseAdapter
{
public:
  BaseAdapter(const AdapterArgs... adapter_args);
};

template<typename..AdapterArgs>
BaseAdapter::BaseAdapter(const AdapterArgs... adapter_args)
{
}

}
