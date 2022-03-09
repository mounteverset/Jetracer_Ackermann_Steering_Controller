#ifndef SIM_FENNEC_HW_INTERFACE_H
#define SIM_FENNEC_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>

namespace fennec_ns
{
/** \brief Hardware interface for a robot */
class SimFennecHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  SimFennecHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);


};  // class

}  // namespace ros_control_boilerplate

#endif
