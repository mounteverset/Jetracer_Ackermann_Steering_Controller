#include "fennec_control/fennec_hw_interface.h"


namespace fennec_ns
{
FennecHWInterface::FennecHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  

}

FennecHWInterface::~FennecHWInterface()
{
  bool close_success = m_i2c_driver->close_i2c_device();
  if (!close_success)
	{
		ROS_INFO_STREAM("FAILED to close I2C device named " << m_i2c_driver->get_device_name());
	}
	else
	{
		ROS_INFO_STREAM("Successfully closed named " << m_i2c_driver->get_device_name() << ", with file descriptor = " << m_i2c_driver->get_file_descriptor());
  }
}

void FennecHWInterface::init()
{
  // Call parent class version of this function
  ros_control_boilerplate::GenericHWInterface::init();

  rosserial_sub = nh_.subscribe("encoder_pulses", 1, &FennecHWInterface::encoderCallback, this);
 
  m_difference_num_pulses_since_last_read = 0;

  bool open_success = m_i2c_driver->open_i2c_device();

  // Display the status
	if (!open_success)
	{
		ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] FAILED to open I2C device named " << m_i2c_driver->get_device_name());
	}
	else
	{
		ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] Successfully opened named " << m_i2c_driver->get_device_name() << ", with file descriptor = " << m_i2c_driver->get_file_descriptor());
	}

  // Encoder Motor Settings
  float motor_freq = 200.0;
  bool result_motor_init = m_pca9685_throttle_driver->initialise_with_frequency_in_hz(motor_freq, false);

  if (!result_motor_init)
	{
		ROS_INFO_STREAM("FAILED - while initialising servo driver with I2C address " << static_cast<int>(m_pca9685_throttle_driver->get_i2c_address()) );
	}


  // Set the servo motor settings etc.
  float servo_freq = 50.0;

  bool result_servo_init = m_pca9685_servo_driver->initialise_with_frequency_in_hz(servo_freq, false);

  if (!result_servo_init)
	{
		ROS_INFO_STREAM("FAILED - while initialising servo driver with I2C address " << static_cast<int>(m_pca9685_servo_driver->get_i2c_address()));
	}

  // Joint Limits
  // std::vector<double> joint_position_lower_limits_;
  // std::vector<double> joint_position_upper_limits_;
  // std::vector<double> joint_velocity_limits_;
  // std::vector<double> joint_effort_limits_;

  joint_position_lower_limits_[1] = 0.3;
  joint_position_upper_limits_[1] = 2.81;
  joint_velocity_limits_[0] = 24.5;

  // Resize vectors
  // joint_position_prev_.resize(num_joints_, 0.0);

  ROS_INFO("FennecHWInterface Ready.");
}

// Get the Pulse Count from ROS Serial on the /encoder_pulses topic
void FennecHWInterface::read(ros::Duration& elapsed_time)
{
  // The want to overwrite the states
  // // States
  // std::vector<double> joint_position_;
  // std::vector<double> joint_velocity_;
  // std::vector<double> joint_effort_;

  // // Commands
  // std::vector<double> joint_position_command_;
  // std::vector<double> joint_velocity_command_;
  // std::vector<double> joint_effort_command_;

  // joint_names -> 0: rear_wheel_joint, 1: steer_wheel_joint

  joint_velocity_[0] = m_difference_num_pulses_since_last_read * PULSE_DISTANCE;

  // Do we need set these 2 values? The joint defintion probably should ignore them
  joint_velocity_[1] = 0;
  joint_position_[0] = 0;

  joint_position_[1] = joint_position_command_[1];

  m_difference_num_pulses_since_last_read = 0;
}


// Write to the PCA9685 the correct channel things
void FennecHWInterface::write(ros::Duration& elapsed_time)
{
  double controller_velocity = joint_velocity_command_[0];
  ROS_INFO_STREAM("The velocity value given by ROS Control: " << controller_velocity);

  // Controlling the servo (PositionJointInterface)
  // The joint_position_command_ gives us a radian to work with 
  // from 0 to 3.141
  // we then translate that to the need pwm for the servo

  // float input = joint_position_command_[1];
  
  // input start = 0
  // input end = 3.14
  // output start = 1000
  // output end = 2000

  pos_jnt_sat_interface_.enforceLimits(elapsed_time);

  float new_servo_pwm = 1000 + ((2000-1000) / (3.14) * (joint_position_command_[1]));

  if (new_servo_pwm > 0)
	{
		if (new_servo_pwm < 1000)
			new_servo_pwm = 1000;
		if (new_servo_pwm > 2000)
			new_servo_pwm = 2000;
	}

  // Call the function to set the desired pulse width
	bool result = m_pca9685_servo_driver->set_pwm_pulse_in_microseconds(servo_channel, new_servo_pwm);


  // Display if an error occurred
	if (!result)
	{
		ROS_INFO_STREAM("FAILED to set pulse width for servo at channel " << static_cast<int>(STEERING_CHANNEL) );
	}


  // DC Motors
  // std::vector<double> joint_velocity_command_
  // unit is rad/s
  // e.g. 6.28/sec = 1 rev/sec
  // this equal 700 pulses by the encoder
  // at 1600hz for the motors the maximum pwm in 0,000625s = 0,625ms = 625 microseconds
  // According to the speed test the motor at maximum pwm_pulse_width 
  // (tested with jetracer python library by nvidia/waveshare) turns 3,9 times/sec 
  // which equal 24,5 rad/s (2pi*3,9)

  float new_motor_pwm = (1 / 24,5 * joint_velocity_command_[1]);


  // unclear wether to use set_pwm_pulse_in_microseconds or set_pwm_pulse
  if (new_motor_pwm > 0)
  {
    m_pca9685_throttle_driver->set_pwm_pulse(1, static_cast<u_int16_t>(0x0FFF * new_motor_pwm), static_cast<u_int16_t>(4096 * (1-new_motor_pwm))); // pwm
    m_pca9685_throttle_driver->set_pwm_pulse(0, 0x0FFF, 0); // full on
    m_pca9685_throttle_driver->set_pwm_pulse(2, 0, 0x0FFF); // full off
    m_pca9685_throttle_driver->set_pwm_pulse(3, 0, 0x0FFF); // full off
    m_pca9685_throttle_driver->set_pwm_pulse(4, static_cast<u_int16_t>(0x0FFF * new_motor_pwm), static_cast<u_int16_t>(0x0FFF * (1-new_motor_pwm))); // pwm
    m_pca9685_throttle_driver->set_pwm_pulse(7, static_cast<u_int16_t>(0x0FFF * new_motor_pwm), static_cast<u_int16_t>(0x0FFF * (1-new_motor_pwm))); // pwm
    m_pca9685_throttle_driver->set_pwm_pulse(6, 0x0FFF, 0);
    m_pca9685_throttle_driver->set_pwm_pulse(5, 0, 0x0FFF);
  }
  else 
  {
    m_pca9685_throttle_driver->set_pwm_pulse(0, static_cast<u_int16_t>(0x0FFF * new_motor_pwm * -1), static_cast<u_int16_t>(0x0FFF * (1 - new_motor_pwm * -1))); 
    m_pca9685_throttle_driver->set_pwm_pulse(1, 0, 0x0FFF); // off
    m_pca9685_throttle_driver->set_pwm_pulse(2, 0x0FFF, 0); // on
    m_pca9685_throttle_driver->set_pwm_pulse(3, static_cast<u_int16_t>(0x0FFF * new_motor_pwm * -1), static_cast<u_int16_t>(0x0FFF * (1 - new_motor_pwm * -1)));
    m_pca9685_throttle_driver->set_pwm_pulse(4, 0, 0x0FFF); // off
    m_pca9685_throttle_driver->set_pwm_pulse(7, static_cast<u_int16_t>(0x0FFF * new_motor_pwm * -1), static_cast<u_int16_t>(0x0FFF * (1 - new_motor_pwm * -1)));
    m_pca9685_throttle_driver->set_pwm_pulse(6, 0, 0x0FFF); // off
    m_pca9685_throttle_driver->set_pwm_pulse(5, 0x0FFF, 0); // on
  }
  
}

void FennecHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
}

// Add the difference between the old m_num_ticks and the msg.data to total_num_ticks
void FennecHWInterface::encoderCallback(const std_msgs::Int16ConstPtr & msg)
{
  // Case for positive overflow 32767 goes to -32768, happens while driving forward
  if ((msg->data - m_old_num_pulses) <= -10000)
  {
    m_difference_num_of_pulses = (msg->data + 65535) - m_old_num_pulses; 
    m_old_num_pulses = msg->data;    
  }
  // Case for negative overflow -32768 goes to 32767, happens while driving backward
  else if ((msg->data - m_old_num_pulses) >= 10000)
  {
    m_difference_num_of_pulses = (msg->data -65535) - m_old_num_pulses;
    m_old_num_pulses = msg->data;
  }
  else
  {
    m_difference_num_of_pulses = msg->data - m_old_num_pulses;
    m_old_num_pulses = msg->data;
  }
  m_difference_num_pulses_since_last_read += m_difference_num_of_pulses;
}

}  // namespace fennec_ns
