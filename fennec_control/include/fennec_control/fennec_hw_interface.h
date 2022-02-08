#ifndef FENNEC_HW_INTERFACE_H
#define FENNEC_HW_INTERFACE_H

// #include <ros.h>
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <std_msgs/Int16.h>
#include "i2c_driver/i2c_driver.h"
#include "pca9685_driver/pca9685.h"
#include <string>

#define PULSE_DISTANCE 0.00031 // The robot moves 0,31mm per pulse
#define PULSE_RADIAN 0.008975979 // The wheel turns 0,0089 rad per pulse
#define STEERING_CHANNEL 0
#define WHEELBASE 0.22
#define WHEEL_RADIUS 0.035
#define DEG_TO_RAD 0.01745329251994329577


#define STEER_JOINT_INPUT_START 204.0 // off_bytes low
#define STEER_JOINT_INPUT_END 450.0 // off_bytes high
#define STEER_JOINT_OUTPUT_START -26.0 // left max steer angle (degree)
#define STEER_JOINT_OUTPUT_END 26.0 // right max steer angle (degree)

#define STEER_JOINT_PULSE_WIDTH_LOW 1000.0 // left
#define STEER_JOINT_PULSE_WIDTH_HIGH 2200.0 //right
#define STEER_JOINT_COMMAND_INPUT_LOW -1.7
#define STEER_JOINT_COMMAND_INPUT_HIGH 1.7

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

  void printCommands();

};  // class

}  // namespace ros_control_boilerplate


#endif
