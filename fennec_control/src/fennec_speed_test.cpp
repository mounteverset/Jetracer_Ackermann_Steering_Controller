#include <ros/ros.h>
#include <std_msgs/Float32.h>

/**
 * @brief A testfile to determine the max speed of the new encoder motors
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("throttle", 1000);

  // ros::Rate loop_rate(0.5);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::Float32 msg;

    msg.data = -1.0;

    ROS_INFO("Publishing %f for 2 Seconds", msg.data);

    chatter_pub.publish(msg);

    ros::spinOnce();

    ros::Duration(2).sleep();

    msg.data = 0.0;

    chatter_pub.publish(msg);

    ROS_INFO("Publishing %f for 15 Seconds", msg.data);

    ros::Duration(15).sleep();

  }


  return 0;
}