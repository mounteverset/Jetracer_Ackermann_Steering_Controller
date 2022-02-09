#include "fennec_control/fennec_hw_interface.h"


namespace fennec_ns
{
FennecHWInterface::FennecHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  

}

FennecHWInterface::~FennecHWInterface()
{ 
  // No force should be applied to the motors after stopping the Hardware Interface
  m_pca9685_throttle_driver->set_all_channels_full_off();
  m_pca9685_servo_driver->set_all_channels_full_off();

  // Close the i2c connection
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
		ROS_INFO_STREAM("FAILED to open I2C device named " << m_i2c_driver->get_device_name());
	}
	else
	{
		ROS_INFO_STREAM("Successfully opened named " << m_i2c_driver->get_device_name() << ", with file descriptor = " << m_i2c_driver->get_file_descriptor());
	}

  // Encoder Motor Settings
  float motor_freq = 600.0;
  bool result_motor_init = m_pca9685_throttle_driver->initialise_with_frequency_in_hz(motor_freq, true);

  if (!result_motor_init)
	{
		ROS_INFO_STREAM("FAILED - while initialising throttle motor driver with I2C address " << static_cast<int>(m_pca9685_throttle_driver->get_i2c_address()) );
	}


  // Set the servo motor settings etc.
  float servo_freq = 50.0;

  bool result_servo_init = m_pca9685_servo_driver->initialise_with_frequency_in_hz(servo_freq, false);

  if (!result_servo_init)
	{
		ROS_INFO_STREAM("FAILED - while initialising servo motor driver with I2C address " << static_cast<int>(m_pca9685_servo_driver->get_i2c_address()));
	}
  else
  {
    ROS_INFO_STREAM("Success - while initialising servo motor driver with I2C address " << static_cast<int>(m_pca9685_servo_driver->get_i2c_address()));
	}
  

  // Joint Limits
  // std::vector<double> joint_position_lower_limits_;
  // std::vector<double> joint_position_upper_limits_;
  // std::vector<double> joint_velocity_limits_;
  // std::vector<double> joint_effort_limits_;

  // joint_position_lower_limits_[0] = 0.1;
  // joint_position_upper_limits_[1] = 3.04;
  // joint_velocity_limits_[0] = 24.5;

  // Resize vectors
  // joint_position_prev_.resize(num_joints_, 0.0);

  ROS_INFO("FennecHWInterface Ready.");
}


void FennecHWInterface::read(ros::Duration& elapsed_time)
{
  ROS_INFO_STREAM("###########################################################################################");
  ROS_INFO_STREAM("###########################  READING THE JOINT INFORMATION  ###############################");
  ROS_INFO_STREAM("###########################################################################################");
  // We want to overwrite the states of the robot stored in these variables
  // std::vector<double> joint_position_;
  // std::vector<double> joint_velocity_;
  // std::vector<double> joint_effort_;

  // Joint index 0: front_steer_joint, 1: rear_wheel_joint
  
  // Rear Joint Readings

  ROS_INFO_STREAM("Elapsed Time since last read: " << elapsed_time.toSec());
  ROS_INFO_STREAM("Reading this Joint_velocity for the rear wheels right now: " << joint_velocity_[0]);
  ROS_INFO_STREAM("Num of pulses since last read: " << m_difference_num_pulses_since_last_read); 
  float distance_since_last_read = m_difference_num_pulses_since_last_read * PULSE_DISTANCE;
  float rad_since_last_read = m_difference_num_pulses_since_last_read * PULSE_RADIAN;
  joint_position_[0] += rad_since_last_read;
  joint_velocity_[0] = rad_since_last_read / elapsed_time.toSec();
  ROS_INFO_STREAM("This is the updated Joint_velocity: " << joint_velocity_[0]);
   
  
  // Steer Joint Readings

  // Read the on and off bytes of the Servo PCA9865 
  // Frequency of the servo is 50 Hz (common Hz for Servos)
  // Pulse Length is 20ms
  // This pulse is divided into 4096 ticks
  // Servos expect a pulse length of 1 milliseconds to 2 milliseconds
  // 1 ms = 0 degree, 2ms = 180 degree
  // this result in the length of the on bytes to be between 204 bytes(4096 ticks/20ms * 1ms) 
  // and 409 bytes (4096 ticks/20ms * 2ms) respectively
  // with this information we can determine the position of the servo motor and write that in the joint_position_[1]
  // The maximum steering angle of the front wheels is around 24 degrees in either direction
  // this equals around 0,4189 radians
  // ROS Control expects the unit to be rad/s which is the angular velocity 
  // 
  // Some important formulas for the calculation of the steering angle based on the read information from the PWM Signal:
  // Steering angle (in degree) = atan(wheelbase / turning_radius)
  // turning_radius = wheelbase / tan(steering_angle) 
  // Angular Velocity = speed (m/s) * tan(steering_angle) (rad) / wheelbase (m)
  // velocity_in_meter_per_sec = (rad_since_last_read / wheel_radius) / elapsed_time.toSec()


  // Determining the steering angle
  uint16_t on_bytes = 0;
  uint16_t off_bytes = 0;
  uint16_t * on_byte_ptr = &on_bytes;
  uint16_t * off_byte_ptr = &off_bytes;
  m_pca9685_servo_driver->get_pwm_pulse(servo_channel, on_byte_ptr, off_byte_ptr);
  ROS_INFO_STREAM("Num of on bytes: " << on_bytes);
  ROS_INFO_STREAM("Num of off_bytes: " << off_bytes);

  // output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)

  float steering_angle = STEER_JOINT_OUTPUT_START + ((STEER_JOINT_OUTPUT_END - STEER_JOINT_OUTPUT_START) / (STEER_JOINT_INPUT_END - STEER_JOINT_INPUT_START)) * (off_bytes - STEER_JOINT_INPUT_START);

  // float steering_angle = 0.234146341 * off_bytes - 71.765853659;

  // We have the problem that when the robot is driving backwards the odometry only cares for the rad/s (the change of rad/s to be specific)
  // for that when we update the joint position and velocity we have to take into account in which direction we are driving
  // If we are driving backwards we want to reverse the rad/s
  // For forward driving we dont have to change anything
  // One big if else block should be sufficient we have the velocity anyway

  float velocity_in_meter_per_sec = (rad_since_last_read * WHEEL_RADIUS) / elapsed_time.toSec();
  float angular_vel = velocity_in_meter_per_sec * tan(steering_angle * DEG_TO_RAD) / WHEELBASE;

  if (steering_angle < -26.0 || steering_angle > 26)
  {
    ROS_INFO_STREAM("This is the calculated steering angle: " << steering_angle * DEG_TO_RAD << ". It lies outside of the possible value range.");
    // joint_position_[0] = 0.0;
  }
  // helps with setting the control command to zero without constantly re-regulating -> hysteresis
  // else if (steering_angle > -0.5 && steering_angle < 0.5)
  // {
  //   joint_position_[1] = 0.0;
  // }
  else
  {
    if (velocity_in_meter_per_sec >= 0)
    {
      joint_position_[1] = steering_angle * DEG_TO_RAD;
    // joint_position_[1] = steering_angle * DEG_TO_RAD;
    }
    if (velocity_in_meter_per_sec < 0)
    {
      joint_position_[1] = steering_angle * DEG_TO_RAD; // * -1?
    }
  }

  if (angular_vel < -2.0 || angular_vel > 2.0)
  {
    ROS_INFO_STREAM("This is the calculated angular_vel: " << angular_vel << ". It lies outside of the possible value range.");
    // joint_velocity_[1] = 0.0;
  }
  else
  {
    if (velocity_in_meter_per_sec < 0)
    {
      joint_velocity_[1] = angular_vel; 
    }
    else 
    {
      joint_velocity_[1] = angular_vel; 
    }


  }

  ROS_INFO_STREAM("steering angle in rad: " << steering_angle * DEG_TO_RAD);
  ROS_INFO_STREAM("steering angle in deg: " << steering_angle);
  ROS_INFO_STREAM("velocity_in_meter_per_sec: " << velocity_in_meter_per_sec);
  ROS_INFO_STREAM("angular_vel: " << angular_vel);
  ROS_INFO_STREAM("joint_velocity[1] (steer_joint): " << joint_velocity_[1]);
  ROS_INFO_STREAM("joint_position[1] (steer_joint): " << joint_position_[1]);

  m_difference_num_pulses_since_last_read = 0;

  ROS_INFO_STREAM("###########################################################################################");
  ROS_INFO_STREAM("#######################  FINISHED READING THE JOINT INFORMATION  ##########################");
  ROS_INFO_STREAM("###########################################################################################");
}


// Write to the PCA9685 the correct channel things
void FennecHWInterface::write(ros::Duration& elapsed_time)
{
  ROS_INFO_STREAM("###########################################################################################");
  ROS_INFO_STREAM("###########################  WRITING THE JOINT INFORMATION  ###############################");
  // ROS_INFO_STREAM("###########################################################################################");
  // double controller_velocity = joint_velocity_command_[0];
  // ROS_INFO_STREAM("The velocity value given by ROS Control: " << controller_velocity);

  // Controlling the servo (PositionJointInterface)
  // The joint_position_command_ gives us a radian to work with 
  // from 0 to 3.141
  // we then translate that to the need pwm for the servo

  // float input = joint_position_command_[1];
  
  // input start = -1.7
  // input end = 1.7
  // output start = 1000
  // output end = 2000
  // output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)

  printCommands();

  pos_jnt_sat_interface_.enforceLimits(elapsed_time);

  // float input_start = -1.0; // right in radians
  // float input_end = 1.0; // left in radians
  // float output_start = 1000; // left
  // float output_end = 2200; // right

  float new_servo_pwm;
  // define a window of no action by having a "deadzone" between two values around zero
  if (joint_position_command_[1] < STEER_JOINT_COMMAND_INPUT_TOLERANCE && joint_position_command_[1] > -STEER_JOINT_COMMAND_INPUT_TOLERANCE)
  {
    new_servo_pwm = 1600;
  }

  if (joint_velocity_command_[0] >= 0)
  {
    // Driving forward, positive steering command equals a positive rad/s
    // Mapping the value range from the input (1.0 to -1.0) to the pwm in microsec (1000 to 2200)
    new_servo_pwm = STEER_JOINT_PULSE_WIDTH_LOW + ((STEER_JOINT_PULSE_WIDTH_HIGH - STEER_JOINT_PULSE_WIDTH_LOW) / (STEER_JOINT_COMMAND_INPUT_HIGH - STEER_JOINT_COMMAND_INPUT_LOW)) * (joint_position_command_[1] - STEER_JOINT_COMMAND_INPUT_LOW);
  }
  else if (joint_velocity_command_[0] < 0)
  {
    // Driving backwards, positive steering command equals a negative rad/s
    // move_base only sends what it wants as output, it does not care how we get there
    // hence, it does not take into account which way we are moving
    // Mapping the value range from the input (1.0 to -1.0) to the pwm in microsec (1000 to 2200), but inverting the input
    new_servo_pwm = STEER_JOINT_PULSE_WIDTH_LOW + ((STEER_JOINT_PULSE_WIDTH_HIGH - STEER_JOINT_PULSE_WIDTH_LOW) / (STEER_JOINT_COMMAND_INPUT_HIGH - STEER_JOINT_COMMAND_INPUT_LOW)) * ((joint_position_command_[1] * -1) - STEER_JOINT_COMMAND_INPUT_LOW);
  

  ROS_INFO_STREAM("The new servo pwm is: " << new_servo_pwm);

  // Trim the values to fit in between 1000 and 2200
  // 1000 == completely left, 2200 == completely right
  if (new_servo_pwm > 0)
	{
		if (new_servo_pwm < STEER_JOINT_PULSE_WIDTH_LOW)
			new_servo_pwm = STEER_JOINT_PULSE_WIDTH_LOW;
		if (new_servo_pwm > STEER_JOINT_PULSE_WIDTH_HIGH)
			new_servo_pwm = STEER_JOINT_PULSE_WIDTH_HIGH;
	}
  // else if (new_servo_pwm == 0)
  // {
  //   new_servo_pwm = 1600;
  // }

  ROS_INFO_STREAM("Writing this value as pwm_pulse_in_microseconds: " << new_servo_pwm);

  // Call the function to set the desired pulse width
	bool result = m_pca9685_servo_driver->set_pwm_pulse_in_microseconds(servo_channel, new_servo_pwm);
  // Display if an error occurred
	if (!result)
	{
		ROS_INFO_STREAM("FAILED to set pulse width for servo at channel " << static_cast<int>(STEERING_CHANNEL) );
	}

  // DC Motors
  // std::vector<double> joint_velocity_command_[0]
  // unit is rad/s
  // e.g. 6.28/sec = 1 rev/sec
  // 1 revolution equal 700 pulses by the encoder
  // at 1600hz for the motors the maximum pwm in 0,000625s = 0,625ms = 625 microseconds
  //
  // According to the speed test the motor at maximum pwm_pulse_width 
  // (tested with jetracer python library by nvidia/waveshare) turns 3,9 times/sec 
  // which equal 24,5 rad/s (2pi*3,9)
  // or 84 cm/s = 0,84 m/s (with wheel circumference = 21,5 cm)

  // float input_start = -24.5; radians
  // float input_end = 24.5; radians
  // float output_start = -1; %pwm rückwarts
  // float output_end = 1; %pwm vorwärts
  // output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
  
  //float new_motor_pwm = -1.0 + ((2.0 / 6.0) * (joint_velocity_command_[0] + 3.0));
  // f(x) = y = 0,040816327 * (x - (-24,5) - 1
  // f(x) = y = 0,040816327 * x
  //float new_motor_pwm = output_start + ((output_end - output_start) / (input_end - input_start)) * (joint_velocity_command_[0] - input_start);

  float new_motor_pwm = -0.040816327 * joint_velocity_command_[0];

  if (new_motor_pwm < REAR_WHEEL_JOINT_MIN_SPEED && new_motor_pwm > -REAR_WHEEL_JOINT_MIN_SPEED)
  {
    if (joint_velocity_command_[0] == 0)
    {
      new_motor_pwm = 0;
    }
    else if (new_motor_pwm < 0)
    {
      new_motor_pwm = -REAR_WHEEL_JOINT_MIN_SPEED;
    }
    else if (new_motor_pwm > 0)
    {
      new_motor_pwm = REAR_WHEEL_JOINT_MIN_SPEED;
    }
  }

  // Wheels are turning the opposite way if not
  ROS_INFO_STREAM("The new rear motor pwm is: " << new_motor_pwm);

  // unclear wether to use set_pwm_pulse_in_microseconds or set_pwm_pulse
  if (new_motor_pwm > 0)
  {
    m_pca9685_throttle_driver->set_pwm_pulse(0, 0, static_cast<u_int16_t>(0x0FFF * new_motor_pwm)); // pwm 
    m_pca9685_throttle_driver->set_pwm_pulse(1, 0, 0x0FFF); // full on
    m_pca9685_throttle_driver->set_pwm_pulse(2, 0, 0); // full off
    m_pca9685_throttle_driver->set_pwm_pulse(3, 0, 0); // full off
    m_pca9685_throttle_driver->set_pwm_pulse(4, 0, static_cast<u_int16_t>(0x0FFF * new_motor_pwm)); // pwm
    m_pca9685_throttle_driver->set_pwm_pulse(7, 0, static_cast<u_int16_t>(0x0FFF * new_motor_pwm)); // pwm
    m_pca9685_throttle_driver->set_pwm_pulse(6, 0, 0x0FFF); // full on
    m_pca9685_throttle_driver->set_pwm_pulse(5, 0, 0);
  }
  else if (new_motor_pwm == 0)
  {
    // full off
    m_pca9685_throttle_driver->set_all_channels_full_off();
    // m_pca9685_throttle_driver->set_pwm_pulse(0, 0, 0x0FFF); //  full off
    // m_pca9685_throttle_driver->set_pwm_pulse(1, 0, 0x0FFF); //  full off
    // m_pca9685_throttle_driver->set_pwm_pulse(2, 0, 0x0FFF); //  full off
    // m_pca9685_throttle_driver->set_pwm_pulse(3, 0, 0x0FFF); //  full off
    // m_pca9685_throttle_driver->set_pwm_pulse(4, 0, 0x0FFF); //  full off
    // m_pca9685_throttle_driver->set_pwm_pulse(7, 0, 0x0FFF); //  full off
    // m_pca9685_throttle_driver->set_pwm_pulse(6, 0, 0x0FFF); //  full off
    // m_pca9685_throttle_driver->set_pwm_pulse(5, 0, 0x0FFF); //  full off
  }
  else if (new_motor_pwm < 0)
  {
    m_pca9685_throttle_driver->set_pwm_pulse(0, 0, static_cast<u_int16_t>(0x0FFF * new_motor_pwm * -1)); 
    m_pca9685_throttle_driver->set_pwm_pulse(1, 0, 0); // off
    m_pca9685_throttle_driver->set_pwm_pulse(2, 0, 0x0FFF); // on -> turns high at 0, turns low at 4095 (last tick)
    m_pca9685_throttle_driver->set_pwm_pulse(3, 0, static_cast<u_int16_t>(0x0FFF * new_motor_pwm * -1));
    m_pca9685_throttle_driver->set_pwm_pulse(4, 0, 0); // off
    m_pca9685_throttle_driver->set_pwm_pulse(7, 0, static_cast<u_int16_t>(0x0FFF * new_motor_pwm * -1));
    m_pca9685_throttle_driver->set_pwm_pulse(6, 0, 0); // off
    m_pca9685_throttle_driver->set_pwm_pulse(5, 0, 0x0FFF); // on
  }  
  ROS_INFO_STREAM("###########################################################################################");
  ROS_INFO_STREAM("##############################  FINISHED WRITING TO JOINTS  ###############################");
  ROS_INFO_STREAM("###########################################################################################");

}

void FennecHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
}

void FennecHWInterface::printCommands()
{   
  // ROS_INFO_STREAM("###########################################################################################");
  ROS_INFO_STREAM("###########################  NEW WRITE COMMANDS BY ROS CONTROL ############################");
  ROS_INFO_STREAM("###########################################################################################");
  // ROS_INFO_STREAM("The length of joint_position_command_ is: " << joint_position_command_.size() << " (Expected is 2)");
  ROS_INFO_STREAM("The joint_position_command_[0] is: " << joint_position_command_[0] << " | Expexted: 0.0");
  ROS_INFO_STREAM("The joint_position_command_[1] is: " << joint_position_command_[1] << " | Expexted: rad/s from -1.0 and 1.0");
  ROS_INFO_STREAM("The joint_velocity_command_[0] is: " << joint_velocity_command_[0] << " | Expected: rad/s -24.5 to 24.5");
  ROS_INFO_STREAM("The joint_velocity_command_[1] is: " << joint_position_command_[1] << " | Expected: rad/s from -1.0 and 1.0");
  ROS_INFO_STREAM("The joint_effort_command_[0] is: " << joint_effort_command_[0] << " | Expected is 0.00");
  ROS_INFO_STREAM("The joint_effort_command_[1] is: " << joint_effort_command_[1] << " | Expected is 0.00");
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
