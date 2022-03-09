#include <fennec_control/sim_fennec_hw_interface.h>

// ROS parameter loading
// #include "rosparam_shortcuts/rosparam_shortcuts.h"

namespace fennec_ns
{
SimFennecHWInterface::SimFennecHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model), name_("sim_fennec_hw_interface")
{
  
}

void SimFennecHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();


  // std::vector<std::string> joint_names_;
  // std::size_t num_joints_;

  // Resize vectors
  joint_position_prev_.resize(num_joints_, 0.0);

  ROS_INFO_NAMED(name_, "SimFennecHWInterface Ready.");
}

void SimFennecHWInterface::read(ros::Duration& elapsed_time)
{
  // No need to read since our write() command populates our state for us

  // // States
  // std::vector<double> joint_position_;
  // std::vector<double> joint_velocity_;
  // std::vector<double> joint_effort_;
  // std::size_t num_joints_;


}

void SimFennecHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);  

  // // Commands
  // std::vector<double> joint_position_command_;
  // std::vector<double> joint_velocity_command_;
  // std::vector<double> joint_effort_command_;
  // std::size_t num_joints_;
}

void SimFennecHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
}

}  // namespace fennec_ns
