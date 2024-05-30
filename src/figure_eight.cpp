#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#define SEC 10

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_publisher");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Twist>("/diff_robot/diff_drive_controller/cmd_vel", 1000);
  ros::Rate loop_rate(10);

  double r = 1.2; // radius of circle
  double v = 0.8; // velocity
  
  ros::Time start_time = ros::Time::now();

  while (ros::ok()) {
    
    geometry_msgs::Twist msg;

    msg.linear.x = v;
    msg.angular.z = v / r;  // Turn left

    // シミュレーション開始からの時間を取得
    int current_time = (int)(ros::Time::now().toNSec() / 1000000000);

    // SEC秒おきに回転方向を切り替える
    if (current_time / SEC % 2 == 0) {
      msg.angular.z =  v / r;  // Turn left
    } else {
      msg.angular.z =  - v / r;  // Turn right
    }
    chatter_pub.publish(msg);

    printf("%d\n", (int)(ros::Time::now().toNSec() / 1000000000));

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
