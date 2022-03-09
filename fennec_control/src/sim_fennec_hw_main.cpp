#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <fennec_control/sim_fennec_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_fennec_hw_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<fennec_ns::SimFennecHWInterface> sim_fennec_hw_interface
    (new fennec_ns::SimFennecHWInterface(nh));
  sim_fennec_hw_interface->init();

  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, sim_fennec_hw_interface);
  control_loop.run();  // Blocks until shutdown signal recieved

  return 0;
}
