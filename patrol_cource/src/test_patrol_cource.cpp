
#include <iostream>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <turtlesim/Pose.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <sstream>


using namespace std;

struct WaypointPose {
  int num;
  double px;
  double py;
  double pz;
  double ox;
  double oy;
  double oz;
  double ow;
};

struct OdomPose {
    double x; // x座標[m] 進行方向
    double y; // y座標[m]
    double theta; // 姿勢 [rad]
};


class DrivingControl {
    private:

        ros::NodeHandle nh;
	  ros::Subscriber odom_sub; 
        // Subscriber to the robot's odom topic
        //ros::Subscriber odom_sub; 
        //ros::Subscriber enemyOdom_sub; 
        OdomPose myPose;
        nav_msgs::Odometry myOdom;
        OdomPose enemyPose;
        nav_msgs::Odometry enemyOdom;
    public:
        //void myOdomCallBack(const nav_msgs::Odometry::ConstPtr& mymsg);
        //void enemyOdomCallBack(const nav_msgs::Odometry::ConstPtr& odom_);
	  void myOdomCallBack(const nav_msgs::Odometry::ConstPtr& mymsg);
	//void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

        DrivingControl();
};

DrivingControl::DrivingControl()
{
	  odom_sub = nh.subscribe("odom", 1, &DrivingControl::myOdomCallBack, this);
        //odom_sub = nh.subscribe("odom", 1, &DrivingControl::myOdomCallback, this);
        //enemyOdom_sub = nh.subscribe("enemy_bot/odom", 1, &DrivingControl::enemyOdomCallback, this);
        //odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &DrivingControl::odomCallback, this);
        //enemyOdom_sub = nh.subscribe<nav_msgs::Odometry>("enemy_bot/odom", 1, &DrivingControl::enemyOdomCallback, this);
}

/*
// Process the incoming laser scan message
void DrivingControl::myOdomCallBack(const nav_msgs::Odometry::ConstPtr& mymsg)
{
	std::string actionMode;
	std::string recoveryMode;
	std::string odomMode;
}
*/

// /自分のodomトピックから自分の位置と姿勢、速度を表示
void DrivingControl::myOdomCallBack(const nav_msgs::Odometry::ConstPtr& mymsg) {
	std::string actionMode;
	std::string recoveryMode;
	std::string odomMode;
}

/*
// /敵のodomトピックから敵の位置と姿勢、速度を表示
void DrivingControl::enemyOdomCallBack(const nav_msgs::Odometry::ConstPtr& _odom) {
    enemyOdom = *_odom;
    //ROS_INFO("Seq: %d", enemyOdom.header.seq);
    //ROS_INFO("/odom Pos (x:%f, y:%f, z:%f)", enemyOdom.pose.pose.position.x,enemyOdom.pose.pose.position.y, enemyOdom.pose.pose.position.z);
    enemyPose.x = enemyOdom.pose.pose.position.x;
    enemyPose.y = enemyOdom.pose.pose.position.y;
    tf::Quaternion q(enemyOdom.pose.pose.orientation.x, enemyOdom.pose.pose.orientation.y, enemyOdom.pose.pose.orientation.z, enemyOdom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    enemyPose.theta = yaw;

    ROS_INFO("/enemy_bot/odom enemyPose (roll:%f, pitch:%f, yaw:%f) ", roll, pitch, yaw);
    ROS_INFO("enemyVel (Linear:%f, Angular:%f)", enemyOdom.twist.twist.linear.x,enemyOdom.twist.twist.angular.z);
}
*/

int main(int argc, char** argv){
  //std::filesystem::path path = std:filesystem::current_path();
  std::string path = ros::package::getPath("generate_waypoints");

  ros::init(argc, argv, "patrol_cource");

  ros::NodeHandle nh;


	// Create new drivingControl object
	DrivingControl drivingControl;
	//Patrol_cource patrol_cource;

  return 0;
}
