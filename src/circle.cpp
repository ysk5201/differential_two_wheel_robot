#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_publisher");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Twist>("/diff_robot/diff_drive_controller/cmd_vel", 1000);
  ros::Rate loop_rate(10);

  double r = 2.0; // radius of circle
  double v = 1.0; // velocity

  while (ros::ok())
  {
    geometry_msgs::Twist msg;

    msg.linear.x = v;
    msg.angular.z = - v / r;

    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
