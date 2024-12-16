#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

class AlohaDriver {
private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  tf::TransformListener listener_;

public:
  AlohaDriver(ros::NodeHandle &nh) {
    nh_ = nh;
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  }

  bool driveForwardOdom(double distance) {
    listener_.waitForTransform("base_footprint", "odom_combined", 
                               ros::Time(0), ros::Duration(1.0));
    
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    listener_.lookupTransform("base_footprint", "odom_combined", 
                              ros::Time(0), start_transform);
    
    geometry_msgs::Twist base_cmd;

    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.25;
    
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok()) {
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      try {
        listener_.lookupTransform("base_footprint", "odom_combined", 
                                  ros::Time(0), current_transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        break;
      }

      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();

      if (dist_moved > distance) done = true;
    }

    if (done) return true;
    return false;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "aloha_driver");
  ros::NodeHandle nh;

  AlohaDriver driver(nh);
  driver.driveForwardOdom(0.5);
}
