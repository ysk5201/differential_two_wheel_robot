#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Float64.h>
#include <time.h>

#define LOOP_RATE 100

const double PI = 3.1415926535897932384626433832795028841971;

const double axel_to_axel_length = 1.0; // 車軸間距離(xacroファイルと合わせる必要あり)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_publisher");
  ros::NodeHandle nh;
  ros::Publisher rear_diff_pub = nh.advertise<geometry_msgs::Twist>("/diff_robot/diff_drive_controller/cmd_vel", 1);
  ros::Rate loop_rate(LOOP_RATE);

  // キーボードで's'を押すとシミュレーション(Publish)を開始させる
  char input;
  std::cout << "Press 's' to start publishing\n";
  std::cin >> input;
  
  double v = 0.5; // 後輪間中点のvelocity

  int count = 0;

  while (ros::ok() && input == 's')
  {
    geometry_msgs::Twist rear_msg;

    count ++;
    double current_time = (double)count/LOOP_RATE;

    double phi = PI / 12; // 仮想的な前輪のステアリング角度
    // double phi = 0.0;
    double r = axel_to_axel_length / tan(phi); // 旋回半径

    // 後輪の制御
    rear_msg.linear.x = v;         // 後輪間中点の移動速度
    rear_msg.angular.z = v / r;  // 旋回角速度

    rear_diff_pub.publish(rear_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
