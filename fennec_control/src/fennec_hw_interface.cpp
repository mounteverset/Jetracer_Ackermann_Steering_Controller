#include <fennec_control/fennec_hw_interface.h>

namespace fennec_ns
{
FennecHWInterface::FennecHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{

}

void FennecHWInterface::init()
{
  // Call parent class version of this function
  ros_control_boilerplate::GenericHWInterface::init();

  // Resize vectors
  // joint_position_prev_.resize(num_joints_, 0.0);

  ROS_INFO("FennecHWInterface Ready.");
}

void FennecHWInterface::read(ros::Duration& elapsed_time)
{
  // No need to read since our write() command populates our state for us
}

void FennecHWInterface::write(ros::Duration& elapsed_time)
{
  
}

void FennecHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  // pos_jnt_sat_interface_.enforceLimits(period);
}

}  // namespace fennec_ns
