#ifndef FENNEC_HW_INTERFACE_H
#define FENNEC_HW_INTERFACE_H

// #include <ros.h>
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include "i2c_driver/i2c_driver.h"
#include "pca9685_driver/pca9685.h"
#include <string>

/**
 * @brief General robot parameters
 * 
 */
#define PULSE_DISTANCE 0.00031 // The robot moves 0,31mm per pulse
#define PULSE_RADIAN 0.008975979 // The wheel turns 0,0089 rad per pulse
#define STEERING_CHANNEL 0
#define WHEELBASE 0.22
#define WHEEL_RADIUS 0.035
#define DEG_TO_RAD 0.01745329251994329577

/**
 * @brief Values used to map to value ranges to one another 
 * 
 */
#define STEER_JOINT_INPUT_START 204.0 // off_bytes low, steering is left
#define STEER_JOINT_INPUT_END 450.0 // off_bytes high, steering is right
#define STEER_JOINT_OUTPUT_START 24.0 // right max steer angle (degree)
#define STEER_JOINT_OUTPUT_END -24.0 // left max steer angle (degree)
#define STEER_JOINT_PULSE_WIDTH_LOW 1000.0 // left
#define STEER_JOINT_PULSE_WIDTH_HIGH 2200.0 //right
#define STEER_JOINT_COMMAND_INPUT_LOW 0.55 // left
#define STEER_JOINT_COMMAND_INPUT_HIGH -0.55 // right
#define STEER_JOINT_COMMAND_INPUT_TOLERANCE 0.001
#define REAR_WHEEL_JOINT_MIN_SPEED 0.2

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

  /**
   * @brief The I2C Bus 1 is used by the PCA9685 boards
   * 
   */
  const char * m_i2c_device_name = "/dev/i2c-1";


  I2C_Driver * m_i2c_driver = new I2C_Driver(m_i2c_device_name);

  /**
   * @brief The default adress for a PCA9685 board
   * 
   */
  const uint8_t m_steering_pca9685_address = 0x40;
  PCA9685 * m_pca9685_servo_driver = new PCA9685(m_i2c_driver, m_steering_pca9685_address);

  /**
   * @brief The adress of the 2nd PCA9685 board
   * 
   */
  const uint8_t m_throttle_pca9685_address = 0x60;
  PCA9685 * m_pca9685_throttle_driver = new PCA9685(m_i2c_driver, m_throttle_pca9685_address);

  /**
   * @brief Needed for calculating the driven rear wheel distance
   * 
   */
  int16_t m_old_num_pulses;

  /**
   * @brief Needed for calculating the driven rear wheel distance
   * 
   */
  int m_difference_num_of_pulses;

  /**
   * @brief Needed for calculating the driven rear wheel distance
   * 
   */
  int m_difference_num_pulses_since_last_read;

  const u_int8_t servo_channel = 0;
  //long m_total_number_of_pulses; 


  /**
   * @brief Subscriber to the /encoder_pulses topic, which is published by the Arduino
   * 
   */
  ros::Subscriber rosserial_sub;

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

  /**
   * @brief Counts the pulses of the encoder. Gets called everytime the rosserial node publishes a new msg
   * 
   * @param msg The current number of pulses, counted by the Arduino, range is from -32768 to 32767
   */
  void encoderCallback(const std_msgs::Int16::ConstPtr & msg);

  /**
   * @brief A helper function to print the ROS Controller Commands for each control loop to the console
   * 
   */
  void printCommands();

  /**
   * @brief Can be changed at runtime with a ROS Param
   * 
   */
  bool logging;

};  // class

}  // namespace ros_control_boilerplate


#endif
