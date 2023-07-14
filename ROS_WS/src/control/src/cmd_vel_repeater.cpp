#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

geometry_msgs::Twist active_vel_twist_msg;

void callback(const geometry_msgs::Twist::ConstPtr &msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "heartbeat_cmd_vel_repeater");
  ros::NodeHandle n;

  ros::Publisher cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Subscriber vel_state_subscriber = n.subscribe<geometry_msgs::Twist>("/vel_state", 10, callback);

  ros::Rate loop_rate(25);
  ROS_INFO("Cmd vel repeater is active.");

  while (ros::ok())
  {
    cmd_vel_publisher.publish(active_vel_twist_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

/**
 * @brief Update the active velocity message
 * @param msg new velocity as Twist
 */
void callback(const geometry_msgs::Twist::ConstPtr &msg)
{
  active_vel_twist_msg = *msg;
}