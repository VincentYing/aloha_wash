#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class AlohaDriver {
private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;

public:
  AlohaDriver(ros::NodeHandle &nh) {
    nh_ = nh;
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }

  bool driveKeyboard() {
    std::cout << "Type a command and then press enter.  "
      "Use '+' to move forward, 'l' to turn left, "
      "'r' to turn right, '.' to exit.\n";

    geometry_msgs::Twist base_cmd;

    char cmd[50];
    while (nh_.ok()) {

      std::cin.getline(cmd, 50);
      if (cmd[0]!='+' && cmd[0]!='l' && cmd[0]!='r' && cmd[0]!='.') {
        std::cout << "unknown command:" << cmd << "\n";
        continue;
      }

      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

      if (cmd[0]=='+') {
        base_cmd.linear.x = 0.25;
      } else if(cmd[0]=='l') {
        base_cmd.angular.z = 0.75;
        base_cmd.linear.x = 0.25;
      } else if(cmd[0]=='r') {
        base_cmd.angular.z = -0.75;
        base_cmd.linear.x = 0.25;
      } else if (cmd[0]=='.') {
        break;
      }

      cmd_vel_pub_.publish(base_cmd);
    }
    return true;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "aloha_driver");
  ros::NodeHandle nh;

  AlohaDriver driver(nh);
  driver.driveKeyboard();
}
