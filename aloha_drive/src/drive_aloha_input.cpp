#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;

#define REPEAT_PUBLISH 5 
#define FIRST_WALL_WINDOW_TIME 50
#define SECOND_WALL_WINDOW_TIME 20
#define FIRST_WALL_TRANSITION_TIME 28
#define SECOND_WALL_TRANSITION_TIME 26
#define LEFT_TURN_TIME 20

class AlohaDriver {
  private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;

  public:
    AlohaDriver(ros::NodeHandle &nh) {
      nh_ = nh;
      cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

    void forward() {
      geometry_msgs::Twist base_cmd;

      base_cmd.linear.x = 1.5;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;

      cmd_vel_pub_.publish(base_cmd);
    }

    void left() {
      geometry_msgs::Twist base_cmd;

      base_cmd.linear.x = 0.25;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0.75;

      cmd_vel_pub_.publish(base_cmd);
    }

    void right() {
      geometry_msgs::Twist base_cmd;

      base_cmd.linear.x = 0.25;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = -0.75;

      cmd_vel_pub_.publish(base_cmd);
    }

    void stop() {
      geometry_msgs::Twist base_cmd;

      base_cmd.linear.x = 0;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;

      int i = 0;
      ros::Rate rate(10);
      while (ros::ok() && i++ < REPEAT_PUBLISH) {
        printf("Time: %d, stop\n", i);
        cmd_vel_pub_.publish(base_cmd);
        rate.sleep();
      }
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "drive_aloha_auto");
  ros::NodeHandle nh;
  string input;

  AlohaDriver driver(nh);

  // Adjust wall alignment
  driver.left();

  // Move to first window
  int i = 0; 
  ros::Rate rate(10);
  while (ros::ok() && i++ < FIRST_WALL_WINDOW_TIME) {
    printf("Time: %d, forward\n", i);
    driver.forward();
    rate.sleep();
  }
  
  driver.stop();

  cout << "============ Press `Enter` to continue drive ...";
  getline(cin, input);

  // Adjust wall alignment
  driver.left();

  // Move to second window
  i = 0;
  while (ros::ok() && i++ < FIRST_WALL_TRANSITION_TIME) {
    printf("Time: %d, forward\n", i);
    driver.forward();
    rate.sleep();
  }

  driver.stop();

  cout << "============ Press `Enter` to continue drive ...";
  getline(cin, input);

  // Move to third window
  i = 0;
  while (ros::ok() && i++ < FIRST_WALL_TRANSITION_TIME) {
    printf("Time: %d, forward\n", i);
    driver.forward();
    rate.sleep();
  }

  driver.stop();

  cout << "============ Press `Enter` to continue drive ...";
  getline(cin, input);

  // Move to fourth window
  i = 0;
  while (ros::ok() && i++ < FIRST_WALL_TRANSITION_TIME) {
    printf("Time: %d, forward\n", i);
    driver.forward();
    rate.sleep();
  }

  driver.stop();

  cout << "============ Press `Enter` to continue drive ...";
  getline(cin, input);

  // Move to second wall
  i = 0;
  while (ros::ok() && i++ < 5) {
    printf("Time: %d, forward\n", i);
    driver.forward();
    rate.sleep();
  }

  // Turn left
  i = 0;
  while (ros::ok() && i++ < LEFT_TURN_TIME) {
    printf("Time: %d, left\n", i);
    driver.left();
    rate.sleep();
  }

  driver.stop();

  cout << "============ Press `Enter` to continue drive ...";
  getline(cin, input);

  // Move to first window
  i = 0;
  while (ros::ok() && i++ < SECOND_WALL_WINDOW_TIME) {
    printf("Time: %d, forward\n", i);
    driver.forward();
    rate.sleep();
  }

  driver.stop();

  cout << "============ Press `Enter` to continue drive ...";
  getline(cin, input);

  // Move to second window
  i = 0;
  while (ros::ok() && i++ < SECOND_WALL_TRANSITION_TIME) {
    printf("Time: %d, forward\n", i);
    driver.forward();
    rate.sleep();
  }

  driver.stop();

  cout << "============ Press `Enter` to continue drive ...";
  getline(cin, input);

  // Move to third window
  i = 0;
  while (ros::ok() && i++ < SECOND_WALL_TRANSITION_TIME) {
    printf("Time: %d, forward\n", i);
    driver.forward();
    rate.sleep();
  }

  driver.stop();
}
