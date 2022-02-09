#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <fennec_control/fennec_hw_interface.h>


/**
 * @brief The main node which executes the hardware interface. This node uses the ros_control_boilerplate_generic HW loop.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fennec_hw_interface");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("Started hw interface");
  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<fennec_ns::FennecHWInterface> fennec_hw_interface
    (new fennec_ns::FennecHWInterface(nh));
  fennec_hw_interface->init();

  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, fennec_hw_interface);
  control_loop.run();  // Blocks until shutdown signal recieved

  return 0;
}
