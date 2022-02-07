#ifndef FENNEC_HW_INTERFACE_H
#define FENNEC_HW_INTERFACE_H

// #include <ros.h>
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <std_msgs/Int16.h>
#include "i2c_driver/i2c_driver.h"
#include "pca9685_driver/pca9685.h"
#include <string>

#define PULSE_DISTANCE 0.03
#define STEERING_CHANNEL 0

namespace fennec_ns
{
/** \brief Hardware interface for a robot */
class FennecHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  FennecHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  virtual ~FennecHWInterface();

  const char * m_i2c_device_name = "/dev/i2c-1";
  I2C_Driver * m_i2c_driver = new I2C_Driver(m_i2c_device_name);
  const uint8_t m_steering_pca9685_address = 0x40;
  PCA9685 * m_pca9685_servo_driver = new PCA9685(m_i2c_driver, m_steering_pca9685_address);
  const uint8_t m_throttle_pca9685_address = 0x60;
  PCA9685 * m_pca9685_throttle_driver = new PCA9685(m_i2c_driver, m_throttle_pca9685_address);
  int16_t m_old_num_pulses;
  int m_difference_num_of_pulses;
  int m_difference_num_pulses_since_last_read;

  const u_int8_t servo_channel = 0;
  //long m_total_number_of_pulses; 

  ros::Subscriber rosserial_sub;

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

  void encoderCallback(const std_msgs::Int16::ConstPtr & msg);

 


  

};  // class

}  // namespace ros_control_boilerplate


#endif
