namespace ilqr_trajectory_tracker
{

class Model
{
};

class BaseModel : public Model
{
public:
  BaseModel();
  virtual void apply_system_dynamics() = 0;
  virtual void get_state_matrix() = 0;
  virtual void get_control_matrix() = 0;
};

BaseModel::BaseModel()
{
}

}
