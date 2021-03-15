
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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
	// Publisher to the robot's mode topic
	//ros::Publisher mode_pub; 
	// Publisher to the robot's velocity command topic
	  ros::Publisher cmd_pub; 
	// Subscriber to the robot's laser scan topic
	//ros::Subscriber laser_sub; 
	// Subscriber to the robot's odom topic
	//ros::Subscriber odom_sub;
	//ros::Subscriber enemyOdom_sub;
	// Subscriber to the robot's goalWaypointPose topic
	//ros::Subscriber pose_sub; 
	// Indicates whether the robot should continue moving
	bool keepMoving; 
	int rotationCounter;
	bool recoveryFlg;
	int recoveryCounter;
	//void moveForward();
	//void moveBackward();
	//void turnToTarget();
	//void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	//void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	//void enemyOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_);
	//void poseCallback(const geometry_msgs::Pose::ConstPtr& pose_);
	//OdomPose myPose;
	//nav_msgs::Odometry myOdom;
	//OdomPose enemyPose;
	//nav_msgs::Odometry enemyOdom;
	geometry_msgs::Twist vel;
public:
	// Tunable parameters
	double FORWARD_SPEED_MPS = 0.2;   //0.2
	double BACKWARD_SPEED_MPS = -0.2; //-0.2
	double ROTATION_SPEED_PIPS = 0.5; //0.5
	//const static double MIN_SCAN_ANGLE_RAD = -30.0/180*3.14;
	//const static double MAX_SCAN_ANGLE_RAD = +30.0/180*3.14;
	//const static double MIN_SCAN_ANGLE_RAD = -45.0/180*3.14;
	//const static double MAX_SCAN_ANGLE_RAD = +45.0/180*3.14;
	//const static double MIN_SCAN_ANGLE_RAD = -60.0/180*3.14;
	//const static double MAX_SCAN_ANGLE_RAD = +60.0/180*3.14;
	//const static double MIN_SCAN_ANGLE_RAD = -110.0/180*3.14;
	//const static double MAX_SCAN_ANGLE_RAD = +110.0/180*3.14;
	double MIN_SCAN_ANGLE_RAD = -1.998;
	double MAX_SCAN_ANGLE_RAD = +1.998;
	double RECOVERY_MIN_SCAN_ANGLE_RAD = -90.0/180*3.14;
	double RECOVERY_MAX_SCAN_ANGLE_RAD = +90.0/180*3.14;
	double FRONT_MIN_SCAN_ANGLE_RAD = -15.0/180*3.14;
	double FRONT_MAX_SCAN_ANGLE_RAD = +15.0/180*3.14;
	// Should be smaller than sensor_msgs::LaserScan::range_max
	float  BAN_PROXIMITY_RANGE_M = 0.25;  //0.15->0.30->0.25
	float  MIN_PROXIMITY_RANGE_M = 0.35;  //0.25->0.35->0.30
	float  BACK_NAVIGATION_RANGE = 0.40;  //0.35->0.40->0.35
	geometry_msgs::Pose goalWaypointPose;

	//void startMoving();
	void turnAround(int direcIndicator);

	void moveForward();
	void moveBackward();

	void setLinearVel(double linear_vel);
	void setAngularVel(double angular_vel);
	void setVel(double linear_vel, double angular_vel);

	//void myOdomCallBack(const nav_msgs::Odometry::ConstPtr& mymsg);
	//void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg);
	//void enemyOdomCallBack(const nav_msgs::Odometry::ConstPtr& odom_);
	//void enemyOdomCallBack(const nav_msgs::Odometry::ConstPtr& odom_);
	//void moveForSecond(double linear_vel, double s);
	//void moveForSecond2(double linear_vel, double s);
	//void moveToDistance(double linear_vel, double dist);

        void startMoving();

	DrivingControl();
	//Patrol_cource();

};


DrivingControl::DrivingControl()
{
	//keepMoving = true;
	keepMoving = false;

	// Advertise a new publisher for the robot's mode topic
	//mode_pub = nh.advertise<std_msgs::String>("mobile_base/command/mode", 10);

	// Advertise a new publisher for the simulated robot's velocity command topic
	cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	// Subscribe to the simulated robot's laser scan topic
	//laser_sub = nh.subscribe("base_scan", 1, &ScanStopper::scanCallback, this);
	//laser_sub = nh.subscribe("scan", 1, &ScanStopper::scanCallback, this);
	//odom_sub = nh.subscribe("odom", 1, &DrivingControl::myOdomCallBack, this);
	//odom_sub = nh.subscribe("odom", 1, &DrivingControl::odomCallback, this);
	//enemyOdom_sub = nh.subscribe("enemy_bot/odom", 1, &DrivingControl::enemyOdomCallBack, this);
	//enemyOdom_sub = nh.subscribe("enemy_bot/odom", 1, &DrivingControl::enemyOdomCallback, this);
	//odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &DrivingControl::odomCallback, this);
	//enemyOdom_sub = nh.subscribe<nav_msgs::Odometry>("enemy_bot/odom", 1, &DrivingControl::enemyOdomCallback, this);
	//pose_sub = nh.subscribe<geometry_msgs::Pose>("goalWaypointPose", 1, &ScanStopper::poseCallback, this);

}

void DrivingControl::startMoving()
{
	//std::string actionMode;
	ros::Rate rate(1);
	ROS_INFO("Start moving");

	// Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
	//while (ros::ok() && keepMoving) {

	//ros::param::set("/recoveryMode","normalRecovery");
	recoveryFlg=true;
	recoveryCounter=0;

	while (ros::ok()) {

		ros::spinOnce(); 
		rate.sleep();
	}
}

// 回転速度の設定
void DrivingControl::setAngularVel(double angular_vel) {
    vel.angular.z = angular_vel;
    cmd_pub.publish(vel);
}

// Send a Rotation command
void DrivingControl::turnAround(int direcIndicator) {

	// The default constructor will set all commands to 0
	double rotationSpeed = ROTATION_SPEED_PIPS;

	/*
	if(direcIndicator<0){
	rotationSpeed = 0.3;
	}else if((direcIndicator-30)>0){
	rotationSpeed = -0.3;
	}else{
	rotationSpeed = 0.3;
	}
	*/

	if(direcIndicator>0){
	rotationSpeed = (-1)*ROTATION_SPEED_PIPS;
	//rotationSpeed = ROTATION_SPEED_PIPS;
	}else{
	rotationSpeed = ROTATION_SPEED_PIPS;
	}

	geometry_msgs::Twist msg; 
	msg.linear.x = 0.0;
	msg.angular.z = rotationSpeed;
	cmd_pub.publish(msg);
}


// 並進速度の設定
void DrivingControl::setLinearVel(double linear_vel) {
    vel.linear.x = linear_vel;
    cmd_pub.publish(vel);
}

// Send a velocity command
void DrivingControl::moveForward() {
	// The default constructor will set all commands to 0
	geometry_msgs::Twist msg; 
	msg.linear.x = FORWARD_SPEED_MPS;
	msg.angular.z = 0.0;
	cmd_pub.publish(msg);
}

// Send a velocity command
void DrivingControl::moveBackward() {
	// The default constructor will set all commands to 0
	geometry_msgs::Twist msg; 
	msg.linear.x = BACKWARD_SPEED_MPS;
	msg.angular.z = 0.0;
	cmd_pub.publish(msg);
}

void DrivingControl::setVel(double linear_vel, double angular_vel = 0) {
    vel.linear.x  = linear_vel;
    vel.angular.z = angular_vel;
    cmd_pub.publish(vel);
}

// /自分のodomトピックから自分の位置と姿勢、速度を表示
/*
void DrivingControl::myOdomCallBack(const nav_msgs::Odometry::ConstPtr& mymsg) {
//void DrivingControl::odomCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
    myOdom = *mymsg;
    ROS_INFO("Seq: %d", myOdom.header.seq);
    ROS_INFO("/odom Pos (x:%f, y:%f, z:%f)", myOdom.pose.pose.position.x,myOdom.pose.pose.position.y, myOdom.pose.pose.position.z);

    myPose.x = myOdom.pose.pose.position.x;
    myPose.y = myOdom.pose.pose.position.y;
    tf::Quaternion q(myOdom.pose.pose.orientation.x, myOdom.pose.pose.orientation.y, myOdom.pose.pose.orientation.z, myOdom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    myPose.theta = yaw;

    ROS_INFO("/odom myPose (roll:%f, pitch:%f, yaw:%f) ", roll, pitch, yaw);
    ROS_INFO("myVel (Linear:%f, Angular:%f)", myOdom.twist.twist.linear.x,myOdom.twist.twist.angular.z);

    //敵との距離を計算する
    double angle_, len_; //目標への移動距離と角度
    //len_ = fabs(sqrt((myPose.x - enemyPose.x)*(myPose.x - enemyPose.x)
    len_ = fabs(sqrt((myPose.x + enemyPose.x)*(myPose.x + enemyPose.x)
                   + (myPose.y - enemyPose.y)*(myPose.y - enemyPose.y)));
    //angle_  = atan2((myPose.y - enemyPose.y), (myPose.x - enemyPose.x));
    angle_  = atan2((myPose.y - enemyPose.y), (myPose.x + enemyPose.x));


     ROS_INFO("### distance form enemy :%f", len_);
     ROS_INFO("### angle with enemy :%f", angle_);
}
*/

// /敵のodomトピックから敵の位置と姿勢、速度を表示
/*
void DrivingControl::enemyOdomCallBack(const nav_msgs::Odometry::ConstPtr& _odom) {
    enemyOdom = *_odom;
    ROS_INFO("/enemy_bot/Seq: %d", enemyOdom.header.seq);
    ROS_INFO("/enemy_bot/odom Pos (x:%f, y:%f, z:%f)", enemyOdom.pose.pose.position.x,enemyOdom.pose.pose.position.y, enemyOdom.pose.pose.position.z);
    enemyPose.x = enemyOdom.pose.pose.position.x;
    enemyPose.y = enemyOdom.pose.pose.position.y;
    tf::Quaternion q(enemyOdom.pose.pose.orientation.x, enemyOdom.pose.pose.orientation.y, enemyOdom.pose.pose.orientation.z, enemyOdom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    enemyPose.theta = yaw;

    ROS_INFO("/enemy_bot/odom enemyPose (roll:%f, pitch:%f, yaw:%f) ", roll, pitch, yaw);
    ROS_INFO("/enemy_bot/Vel (Linear:%f, Angular:%f)", enemyOdom.twist.twist.linear.x,enemyOdom.twist.twist.angular.z);
}
*/

/*
// Called once when the goal completes
//void doneCb(const actionlib::SimpleClientGoalState& state)
void doneCb()
{
    ROS_INFO("Got Result");
  //ROS_INFO("Finished in state [%s]", state.toString().c_str());
  //ROS_INFO("Answer: %i", result->sequence.back());
  //ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
//void feedbackCb()
{
    move_base_msgs::MoveBaseFeedback _feedback;
    _feedback = *feedback;
    //ROS_INFO("Got Feedback");
    //ROS_INFO("Got Feedback x= :%f",feedback.status);
    //ROS_INFO("Got Feedback x= :%f",feedback.base_position.pose.position.x);
    //ROS_INFO("Got Feedback y= :%f",feedback.feedback.base_position.pose.position.x);
    //ROS_INFO("Got Feedback x= :%f",*feedback->base_position->pose->position->x);
    //ROS_INFO("Got Feedback y= :%f",feedback->base_position->pose->position->x);
    ros::spinOnce(); 
    //rate.sleep();
    std::string actionMode;
    if(ros::param::get("/actionMode",actionMode)){

        if(actionMode.compare("aimEnemy")==0){
            //ac.cancelGoal();
            //goalCancelFlg=true;
        }
    }
}
*/


int main(int argc, char** argv){
  //std::filesystem::path path = std:filesystem::current_path();
  std::string path = ros::package::getPath("generate_waypoints");

  //WaypointPose way_point[] = {{-0.996, 1.496, 0.898,0.440},{-0.429, 2.208, 0.027,1.000},{-1.516, 2.128, -0.719,0.695},{0.028, 0.017, -0.715,0.699}};

  ros::Publisher  pub_goalWaypointPose;

  ros::init(argc, argv, "patrol_cource");

  ros::NodeHandle nh;
  //pub_goalWaypointPose= nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("goalWaypointPose", 10, true);
  pub_goalWaypointPose= nh.advertise<geometry_msgs::Pose>("goalWaypointPose", 10, true);


	// Create new drivingControl object
	DrivingControl drivingControl;
	//drivingControl.startMoving();

  // チェックポイントをway_point[]に入力していく
  std::string line="";
  float px=0.0, py=0.0, pz=0.0, ox=0.0, oy=0.0, oz=0.0, ow=0.0;
  int i=0;
  //ifstream ifs{};

  //======== get waypoints for patrol ==============

  //open patrol_waypoints file
  //ifstream ifsForPatrol(path.string()+"/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsForPatrol.csv");
  ifstream ifsForPatrol(path+"/scripts/waypointsForPatrol.csv");
  //ifs(path.string()+"/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsForPatrol.csv");
  if(ifsForPatrol.fail()){
    ROS_INFO( "File do not exist.");
    exit(0);
  }

  //create wayPoints array
  WaypointPose waypointsForPatrol[50];

  getline(ifsForPatrol, line); //read&discard header line
  int waypointIndexPatrol=0;
  int goalWayPointIndex=0;

  i=0;
  while(getline(ifsForPatrol, line)) {
    //ROS_INFO("%s",line);
    sscanf(line.data(), "%d,%f,%f,%f,%f,%f,%f,%f",&waypointIndexPatrol,&px,&py,&pz,&ox,&oy,&oz,&ow);
    waypointsForPatrol[i].px = px;
    waypointsForPatrol[i].py = py;
    waypointsForPatrol[i].pz = pz;
    waypointsForPatrol[i].ox = ox;
    waypointsForPatrol[i].oy = oy;
    waypointsForPatrol[i].oz = oz;
    waypointsForPatrol[i].ow = ow;
    
    //ROS_INFO("way_point[%d]: %f, %f, %f, %f,%f, %f, %f", waypointIndexPatrol,px,py,pz,ox,oy,oz,ow);
    ROS_INFO("waypointsForPatrol[%d]: %f, %f, %f, %f,%f, %f, %f", i,px,py,pz,ox,oy,oz,ow);
    i++;
  }
  //int lineNum=i;   
  int patrolWaypointsNum=i;   
  i=0;
  //if(ifs.is_open()) ifs.close;
  ifsForPatrol.close();


/*▲▲▲

  //======== get waypoints To Empty Can ==============
  //open waypoints csv file for moving to EmptyCan
  //ifstream ifsToEmptyCan(path.string()+"/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToEmptyCan.csv");
  ifstream ifsToEmptyCan(path+"/scripts/waypointsToEmptyCan.csv");
  //ifsToEmptyCan(path.string()+"/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToEmptyCan.csv");
  if(ifsToEmptyCan.fail()){
    ROS_INFO( "File do not exist.");
    exit(0);
  }

  //create wayPoints array
  WaypointPose waypointsToEmptyCan[50];

  // チェックポイントをway_point[]に入力していく
  //float px=0.0, py=0.0, pz=0.0, ox=0.0, oy=0.0, oz=0.0, ow=0.0;
  //std::string line="";
  getline(ifsToEmptyCan, line); //read&discard header line
  int waypointIndexEmptyCan=0;
  i=0;
  while(getline(ifsToEmptyCan, line)) {
    //ROS_INFO("%s",line);
    sscanf(line.data(), "%d,%f,%f,%f,%f,%f,%f,%f",&waypointIndexEmptyCan,&px,&py,&pz,&ox,&oy,&oz,&ow);
    waypointsToEmptyCan[i].px = px;
    waypointsToEmptyCan[i].py = py;
    waypointsToEmptyCan[i].pz = pz;
    waypointsToEmptyCan[i].ox = ox;
    waypointsToEmptyCan[i].oy = oy;
    waypointsToEmptyCan[i].oz = oz;
    waypointsToEmptyCan[i].ow = ow;
    
    //ROS_INFO("way_point[%d]: %f, %f, %f, %f,%f, %f, %f", waypointIndexEmptyCan,px,py,pz,ox,oy,oz,ow);
    ROS_INFO("waypointsToEmptyCan[%d]: %f, %f, %f, %f,%f, %f, %f", i,px,py,pz,ox,oy,oz,ow);
    i++;
  }
  //int lineNum=i;   
  int waypointsNumToEmptyCan=i;   
  i=0;
  ifsToEmptyCan.close();

  //======== get waypoints To Garbage Box ==============

  //open waypoints csv file for moving to GarbageBox
  //ifstream ifsToGarbageBox(path.string()+"/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToGarbageBox.csv");
  ifstream ifsToGarbageBox(path+"/scripts/waypointsToGarbageBox.csv");
  //ifsToGarbageBox(path.string()+"/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToGarbageBox.csv");
  if(ifsToGarbageBox.fail()){
    ROS_INFO( "File do not exist.");
    exit(0);
  }

  //create wayPoints array
  WaypointPose waypointsToGarbageBox[50];


  // チェックポイントをway_point[]に入力していく
  //std::string line="";
  //float px=0.0, py=0.0, pz=0.0, ox=0.0, oy=0.0, oz=0.0, ow=0.0;
  getline(ifsToGarbageBox, line); //read&discard header line
  int waypointIndexGarbageBox=0;
  i=0;
  while(getline(ifsToGarbageBox, line)) {
    //ROS_INFO("%s",line);
    sscanf(line.data(), "%d,%f,%f,%f,%f,%f,%f,%f",&waypointIndexGarbageBox,&px,&py,&pz,&ox,&oy,&oz,&ow);
    waypointsToGarbageBox[i].px = px;
    waypointsToGarbageBox[i].py = py;
    waypointsToGarbageBox[i].pz = pz;
    waypointsToGarbageBox[i].ox = ox;
    waypointsToGarbageBox[i].oy = oy;
    waypointsToGarbageBox[i].oz = oz;
    waypointsToGarbageBox[i].ow = ow;
    
    //ROS_INFO("way_point[%d]: %f, %f, %f, %f,%f, %f, %f", waypointIndexGarbageBox,px,py,pz,ox,oy,oz,ow);
    ROS_INFO("waypointsToGarbageBox[%d]: %f, %f, %f, %f,%f, %f, %f", i,px,py,pz,ox,oy,oz,ow);
    i++;
  }
  //int lineNum=i;   
  int waypointsNumToGarbageBox=i;   
  i=0;
  ifsToGarbageBox.close();


  //======== get waypoints To Dock Station ==============

  //open waypoints csv file for moving to Dock Station
  //ifstream ifsToDockStation(path.string()+"/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToDockStation.csv");
  ifstream ifsToDockStation(path+"/scripts/waypointsToDockStation.csv");
  //ifs(path.string()+"/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToDockStation.csv");
  if(ifsToDockStation.fail()){
    ROS_INFO( "File do not exist.");
    exit(0);
  }

  //create wayPoints array
  WaypointPose waypointsToDockStation[50];

  // チェックポイントをway_point[]に入力していく
  //std::string line="";
  //float px=0.0, py=0.0, pz=0.0, ox=0.0, oy=0.0, oz=0.0, ow=0.0;
  getline(ifsToDockStation, line); //read&discard header line
  int waypointIndexDockStation=0;
  i=0;
  while(getline(ifsToDockStation, line)) {
    //ROS_INFO("%s",line);
    sscanf(line.data(), "%d,%f,%f,%f,%f,%f,%f,%f",&waypointIndexDockStation,&px,&py,&pz,&ox,&oy,&oz,&ow);
    waypointsToDockStation[i].px = px;
    waypointsToDockStation[i].py = py;
    waypointsToDockStation[i].pz = pz;
    waypointsToDockStation[i].ox = ox;
    waypointsToDockStation[i].oy = oy;
    waypointsToDockStation[i].oz = oz;
    waypointsToDockStation[i].ow = ow;
    
    //ROS_INFO("way_point[%d]: %f, %f, %f, %f,%f, %f, %f", waypointIndexDockStation,px,py,pz,ox,oy,oz,ow);
    ROS_INFO("waypointsToDockStation[%d]: %f, %f, %f, %f,%f, %f, %f", i,px,py,pz,ox,oy,oz,ow);
    i++;
  }
  //int lineNum=i;   
  int waypointsNumToDockStation=i;   
  i=0;
  ifsToDockStation.close();

▲▲▲
*/

  //======== Driving Control ==============
  //======== Driving Control for Patrol ==============
 
  // アクションクライアンを作成。1番目の引数は接続するアクションサーバー名。
  // アクションサーバーが立ち上がっていないとだめ。
  // ２番目の引数はtrueならスレッドを自動的に回す(ros::spin()。
  MoveBaseClient ac("move_base", true);
  // アクションサーバーが起動するまで待つ。引数はタイムアウトする時間(秒）。
  // この例では５秒間待つ(ブロックされる)
  while(!ac.waitForServer(ros::Duration(60.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("The server comes up");
  move_base_msgs::MoveBaseGoal goal;
  // base_link座標系（ロボット座標系)
  //goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.frame_id = "map";
  // 現在時刻                                                                       
  goal.target_pose.header.stamp = ros::Time::now();


  //======== Driving Control for Patrol ==============
  std::string actionMode;
  std::string searchEnemyResult;
  std::string aimEnemyTargetResult;

  std::string patrolSubMode;
  std::string actionDuration;
  std::string patrolResult;
	geometry_msgs::Pose goalWaypointPose;

  //int waypointIndex = 0;
  waypointIndexPatrol = 0;
  goalWayPointIndex=waypointIndexPatrol;
/*
▲▲▲
  waypointIndexEmptyCan=0;
  waypointIndexGarbageBox=0;
  waypointIndexDockStation=0;
▲▲▲
*/
  bool goalCancelFlg=false;
  int direcIndicator = 0;

  bool sendGoalFlgAtSearchEnemy=false;
  bool sendGoalFlgAtAimEnemyTarget=false;

  ros::Rate rate(1);
/*
			  //少し前に出て、ターゲットを探しやすいように姿勢を調整する
			  for(int i=1;i<3;i++){
				drivingControl.moveForward();
				rate.sleep();
			  }
			  //右へ回転し、右側のフィールドターゲットを探して点数を取得する
			for(int i=1;i<4;i++){
				ROS_INFO("--------waypoint(1)--------- ");
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			}

			  //左へ回転し、左側のフィールドターゲットを探して点数を取得する
			for(int i=1;i<6;i++){
				direcIndicator = -10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			}
			  //右へ回転し、１つ目のwaypointへ走行しやすいように姿勢を調整する
			for(int i=1;i<3;i++){
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			}
*/

  //最初のwaypointで事前処理をしないように、サブ走行モード(patrolSubMode)を"movingToGoal"に設定する
  //ros::param::set("patrolSubMode","preAdjustment");
  ros::param::set("patrolSubMode","movingToGoal");

  while (ros::ok()) {
      //ros::spinOnce(); 
      //rate.sleep();

      if(ros::param::get("/actionMode",actionMode)){

          if(actionMode.compare("patrol")==0){
              if(goalCancelFlg) goalCancelFlg=false;
		ros::param::get("/patrolSubMode",patrolSubMode);

              //goal.task_id = 1; 

              // ROSではロボットの進行方向がx座標、左方向がy座標、上方向がz座標
              goal.target_pose.pose.position.x =  waypointsForPatrol[waypointIndexPatrol].px;
              goal.target_pose.pose.position.y =  waypointsForPatrol[waypointIndexPatrol].py;
              goal.target_pose.pose.orientation.z = waypointsForPatrol[waypointIndexPatrol].oz; 
              goal.target_pose.pose.orientation.w = waypointsForPatrol[waypointIndexPatrol].ow; 

	      goalWaypointPose=goal.target_pose.pose;

              ROS_INFO("Sending goal: No.%d", waypointIndexPatrol);
		//リカバリ走行と情報共有するため
	      ros::param::set("patrolWaypointIndex",waypointIndexPatrol);

//2021 for roboticsHubChanllenge
              // サーバーにgoalを送信
              //ac.sendGoal(goal);

		//if (waypointIndexPatrol>=patrolWaypointsNum) waypointIndexPatrol=0;

		if(waypointIndexPatrol==0){
			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する

			//アクション走行の合間に敵探しをこまめに実施する。
			//sendGoalFlgAtAimEnemyTarget = false;
			//ros::param::set("actionMode","aimEnemyTarget");

			}


			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");
              		// サーバーにgoalを送信し、次のwaypointへ走行させる
              		//ac.sendGoal(goal);
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
			  		// waypointIndexインクリメント
			  		waypointIndexPatrol++;
					ros::param::set("patrolSubMode","preAdjustment");

					ac.sendGoal(goal);
				}
			}

		}else if(waypointIndexPatrol==1){
		  //１つ目のWaypointに到着


			//★前方方向　敵探し
			//★赤ランプで探す　

			//★敵が右方向にいるなら、waypointIndexPatrol+1へ走行し、SubModeをfight_enemyにする

			//★敵が右左方向にいるなら、waypointIndexPatrol+2へ走行し、SubModeをfight_enemyにする

			//★敵がいないなら、waypointIndexPatrol+2へ走行し、SubModeをfight_enemyにする

			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する

				//アクション走行の合間に敵探しをこまめに実施する。
				if(ros::param::get("/actionMode",actionMode)){
        				if(actionMode.compare("patrol")==0){
						//sendGoalFlgAtAimEnemyTarget = false;
						ros::param::set("actionMode","aimEnemyTarget");
						//rate.sleep();
					}
				}
/*
*/

			  //フィールドターゲットを探して点数を取得する
				//ros::param::set("actionMode","aimTarget");
			  //真正面のフィールドターゲットを探して点数を取得する
			  //右へ回転し、右側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<5;i++){
				ROS_INFO("--------waypoint(1)--------- ");
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }

			//★右方向　敵探し
			//★赤ランプで探す　
			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}
/*
*/
			  //左へ回転し、左側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<7;i++){
				direcIndicator = -10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }

			//★左方向　敵探し
			//★赤ランプで探す　
			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}
/*
*/
			  //右へ回転し、２つ目のwaypointへ走行しやすいように姿勢を調整する
			  for(int i=1;i<5;i++){
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }

			//★前方方向　敵探し
			//★赤ランプで探す　
			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}

			}


			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");

	  		// waypointIndexインクリメント
	  		//waypointIndexPatrol++;
              		// サーバーにgoalを送信し、次のwaypointへ走行させる
              		//ac.sendGoal(goal);
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
			  		// waypointIndexインクリメント
			  		waypointIndexPatrol++;
					ros::param::set("patrolSubMode","preAdjustment");

					ac.sendGoal(goal);
				}
			}



		}else if(waypointIndexPatrol==2){
		  //２つ目のWaypointに到着



			//★前方方向　敵探し
			//★赤ランプで探す　（遠距離、広い角度で探せる、障害物あっても良い）▲直近距離に弱い <0.6m　▲真横から後部方法には検出できない

			//カメラで敵探し
			//見つかった場合：　searchEnemyResultパラメータ確認＋Topic受信（前方, 右、左）
			//見つからない場合：　searchEnemyResultパラメータ確認
			//ros::param::set("actionMode","searchEnemy");

			//★敵が右方向にいるなら、waypointIndexPatrol+1へ走行し、SubModeをfight_enemyにする

			//★敵が右左方向にいるなら、waypointIndexPatrol+2へ走行し、SubModeをfight_enemyにする

			//★赤ランプで敵を見つからなかった場合　⇒★緑ランプで敵を探す（近距離、前方方向）
			//★緑ランプで敵を探す（近距離、前方方向）　▲遠距離や横方向に弱い、障害物で遮る、▲ロボットが真正面から向かってきた場合に検出できない


			//★カメラで敵を見つからなかった場合⇒★LiDARで敵を探す
			////★LiDARで敵を探す（0m〜3.5mの範囲、360度、直視）　▲障害部で遮る、▲ノイズがある（条件判定：>0.1m,且つ <3.5m）
			////見つかった場合：　scanEnemyResultパラメータ確認＋Topic受信（角度：radで前方、後方、 右、左を判定できる）
			////見つからない場合：　scanEnemyResultパラメータ確認（停止しているか、障害物の後ろにいるか）

			//★敵が左前部方向にいるなら、waypointIndexPatrol+1へ走行し、SubModeをfight_enemyにする

			//★敵が右前部方向にいるなら、waypointIndexPatrol+2へ走行し、SubModeをfight_enemyにする

			//★敵が左後部方向にいるなら、waypointIndexPatrol+3へ走行し、SubModeをfight_enemyにする

			//★敵が右後部方向にいるなら、waypointIndexPatrol+4へ走行し、SubModeをfight_enemyにする



			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する


			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}
			}


			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");

	  		// waypointIndexインクリメント
	  		//waypointIndexPatrol++;
              		// サーバーにgoalを送信し、次のwaypointへ走行させる
              		//ac.sendGoal(goal);
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
			  		// waypointIndexインクリメント
			  		waypointIndexPatrol++;
					ac.sendGoal(goal);
				}
			}



		}else if(waypointIndexPatrol==3){
			//３つ目のWaypointに到着


			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する
			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}

			}


			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");

	  		// waypointIndexインクリメント
	  		//waypointIndexPatrol++;
              		// サーバーにgoalを送信し、次のwaypointへ走行させる
              		//ac.sendGoal(goal);
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
			  		// waypointIndexインクリメント
			  		waypointIndexPatrol++;
					ac.sendGoal(goal);
				}
			}


		}else if(waypointIndexPatrol==4){
			//４つ目のWaypointに到着
			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する


			  //フィールドターゲットを探して点数を取得する
				//ros::param::set("actionMode","aimTarget");
			  //真正面のフィールドターゲットを探して点数を取得する
			  //右へ回転し、右側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<4;i++){
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }

			//★右方向　敵探し
			//★赤ランプで探す　
			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}
/*
*/
			  //左へ回転し、左側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<5;i++){
				direcIndicator = -10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }


			//★左方向　敵探し
			//★赤ランプで探す　

			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}
			}



			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");

	  		// waypointIndexインクリメント
	  		//waypointIndexPatrol++;
              		// サーバーにgoalを送信し、次のwaypointへ走行させる
              		//ac.sendGoal(goal);
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
			  		// waypointIndexインクリメント
			  		waypointIndexPatrol++;
					ac.sendGoal(goal);
				}
			}


		}else if(waypointIndexPatrol==5){
			//５つ目のWaypointに到着


			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する


			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}
			}


			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");

	  		// waypointIndexインクリメント
	  		//waypointIndexPatrol++;
              		// サーバーにgoalを送信し、次のwaypointへ走行させる
              		//ac.sendGoal(goal);
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
			  		// waypointIndexインクリメント
			  		waypointIndexPatrol++;
					ac.sendGoal(goal);
				}
			}


		}else if(waypointIndexPatrol==6){
			//６つ目のWaypointに到着
			if(patrolSubMode.compare("preAdjustment")==0){
			  //事前処理
			  //ターゲット捜索し、点数を取得する
			  //次のwaypointへ走行しやすいように姿勢を調整する


			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}
			}


			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");

	  		// waypointIndexインクリメント
	  		//waypointIndexPatrol++;
              		// サーバーにgoalを送信し、次のwaypointへ走行させる
              		//ac.sendGoal(goal);
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
			  		// waypointIndexインクリメント
			  		waypointIndexPatrol++;
					ac.sendGoal(goal);
				}
			}


		}else if(waypointIndexPatrol==7){
			//７つ目のWaypointに到着
			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する

			  //フィールドターゲットを探して点数を取得する
				//ros::param::set("actionMode","aimTarget");
			  //真正面のフィールドターゲットを探して点数を取得する
			  //右へ回転し、右側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<4;i++){
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }

			//★右方向　敵探し
			//★赤ランプで探す　
			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}
/*
*/
			  //左へ回転し、左側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<7;i++){
				direcIndicator = -10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }


			//★左方向　敵探し
			//★赤ランプで探す　
			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}

			  //右へ回転し、走行しやすい方向に調整する
			  for(int i=1;i<4;i++){
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }

			//★右方向　敵探し
			//★赤ランプで探す　
			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}



			}


			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");

	  		// waypointIndexインクリメント
	  		//waypointIndexPatrol++;
              		// サーバーにgoalを送信し、次のwaypointへ走行させる
              		//ac.sendGoal(goal);
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
			  		// waypointIndexインクリメント
			  		waypointIndexPatrol++;
					ac.sendGoal(goal);
				}
			}



		}else if(waypointIndexPatrol==8){
			//８つ目のWaypointに到着
			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する

			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}
			}

			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");

	  		// waypointIndexインクリメント
	  		//waypointIndexPatrol++;
              		// サーバーにgoalを送信し、次のwaypointへ走行させる
              		//ac.sendGoal(goal);
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
			  		// waypointIndexインクリメント
			  		waypointIndexPatrol++;
					ac.sendGoal(goal);
				}
			}

		//}else if (waypointIndexPatrol>=patrolWaypointsNum) {
		}else if(waypointIndexPatrol==9){
			//９つ目のWaypointに到着
			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する
			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}

			}

			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");

	  		// waypointIndexインクリメント
	  		//waypointIndexPatrol++;
              		// サーバーにgoalを送信し、次のwaypointへ走行させる
              		//ac.sendGoal(goal);
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
			  		// waypointIndexインクリメント
			  		waypointIndexPatrol++;
					ac.sendGoal(goal);
				}
			}


		}else if(waypointIndexPatrol==10){
			//１０つ目のWaypointに到着
			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する


			  //フィールドターゲットを探して点数を取得する
				//ros::param::set("actionMode","aimTarget");
			  //真正面のフィールドターゲットを探して点数を取得する
			  //右へ回転し、右側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<4;i++){
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }

			//★右方向　敵探し
			//★赤ランプで探す　
			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}
/*
*/
			  //左へ回転し、左側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<5;i++){
				direcIndicator = -10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }


			//★左方向　敵探し
			//★赤ランプで探す　


			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}
			}


			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");

	  		// waypointIndexインクリメント
	  		//waypointIndexPatrol++;
              		// サーバーにgoalを送信し、次のwaypointへ走行させる
              		//ac.sendGoal(goal);
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
			  		// waypointIndexインクリメント
			  		waypointIndexPatrol++;
					ac.sendGoal(goal);
				}
			}


		}else if(waypointIndexPatrol==11){
			//１１つ目のWaypointに到着
			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する
			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}

			}

			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");

	  		// waypointIndexインクリメント
	  		//waypointIndexPatrol++;
              		// サーバーにgoalを送信し、次のwaypointへ走行させる
              		//ac.sendGoal(goal);
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
			  		// waypointIndexインクリメント
			  		waypointIndexPatrol++;
					ac.sendGoal(goal);
				}
			}


		}else if (waypointIndexPatrol>=12) {
			//１２つ目のWaypointに到着


			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する

			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}

			}


			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");

	  		// waypointIndex初期化
	  		//waypointIndexPatrol=0;
              		// サーバーにgoalを送信し、次のwaypointへ走行させる
              		//ac.sendGoal(goal);
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
			  		// waypointIndex初期化
			  		waypointIndexPatrol=0;
					//ac.sendGoal(goal);
				}
			}

		}

              	// 結果が返ってくるまで30.0[s] 待つ。ここでブロックされる。
              	//bool succeeded = ac.waitForResult(ros::Duration(120.0));
              	//ac.waitForResult(ros::Duration(30.0));
		
		double actionTime=2.0;
		//double actionTime=30.0;
      		if(ros::param::get("/actionDuration",actionDuration)){
          		if(actionDuration.compare("middleTime")==0){
			  actionTime=4.0;
			}
          		if(actionDuration.compare("longTime")==0){
			  //actionDuartion: longTime
			  actionTime=6.0;
			}
		}

              	ac.waitForResult(ros::Duration(actionTime));

              	// 結果を見て、成功ならSucceeded、失敗ならFailedと表示
              	actionlib::SimpleClientGoalState state = ac.getState();

		patrolResult=state.toString().c_str();
          	if(patrolResult.compare("SUCCEEDED")==0){
	       		ROS_INFO("Succeeded: waypointIndexPatrol No.%d (%s)",goalWayPointIndex, state.toString().c_str());
			goalWayPointIndex=waypointIndexPatrol;
               		ROS_INFO("nextWayPointIndex: No.%d",goalWayPointIndex);
			//ros::param::set("patrolWaypointIndex",waypointIndexPatrol);
			//サブ走行モード設定：patrolSubMode
			//ros::param::set("patrolSubMode","preAdjustment");
			//ros::param::set("patrolSubMode","movingToGoal");

			//アクション走行の合間に敵探しをこまめに実施する。
			//sendGoalFlgAtAimEnemyTarget = false;
			//ros::param::set("actionMode","aimEnemyTarget");

			//scanEnemyノードを動作させるために、走行停止してから５秒待つ
			//drivingControl.setVel(0.0,0.0);
			//rate.sleep();
			//rate.sleep();
			//rate.sleep();
			//rate.sleep();
			//rate.sleep();
			//rate.sleep();
			//ros::param::set("actionMode","scanEnemy");

		}else {
                	ROS_INFO("Failed: waypointIndexPatrol No.%d (%s)",(waypointIndexPatrol-1), state.toString().c_str());
			//サブ走行モード設定(patrolSubMode)を変更しない：movingToGoalのまま
			ros::param::set("patrolSubMode","movingToGoal");

			//次のwaypointへ進まないようにする
			waypointIndexPatrol--;
			goalWayPointIndex=waypointIndexPatrol;
               		//ROS_INFO("nextWayPointIndex: No.%d",goalWayPointIndex);

			//リカバリ走行と情報共有するため
			ros::param::set("patrolWaypointIndex",waypointIndexPatrol);
			pub_goalWaypointPose.publish(goalWaypointPose);

			//アクション失敗時に、リカバリ走行させる
			//ros::param::set("originalActionMode","patrol");
			//ros::param::set("actionMode","naviRecovery");

			//アクション走行の合間に敵探しをこまめに実施する。
			if(ros::param::get("/actionMode",actionMode)){
        			if(actionMode.compare("patrol")==0){
					//sendGoalFlgAtAimEnemyTarget = false;
					ros::param::set("actionMode","aimEnemyTarget");
					//rate.sleep();
				}
			}

			//アクション走行の合間に敵探しをこまめに実施する。
			//ros::param::set("actionMode","searchEnemy");

			//scanEnemyノードを動作させるために、走行停止してから５秒待つ
			//drivingControl.setVel(0.0,0.0);
			//rate.sleep();
			//rate.sleep();
			//rate.sleep();
			//rate.sleep();
			//rate.sleep();
			//rate.sleep();
			//ros::param::set("actionMode","scanEnemy");

            	}
	  } else{
             //abort
             // if(!goalCancelFlg){
             //   ac.cancelGoal();
             //   goalCancelFlg=true;
  	     //   waypointIndexPatrol--;
             // }
          }

          if(actionMode.compare("scanEnemy")==0){

	  }


          if(actionMode.compare("searchEnemy")==0){

/*
		//敵探しするために５秒待つ
		rate.sleep();
		rate.sleep();
		rate.sleep();

		//敵探しの結果を確認する

      		if(ros::param::get("/searchEnemyResult",searchEnemyResult)){
          		if(searchEnemyResult.compare("fail")==0){
                                ROS_INFO("!!!***!!! back to patrol mode (1)");
				ros::param::set("actionMode","patrol");
              		  	// サーバーにgoalを送信
              		 	if (sendGoalFlgAtSearchEnemy) ac.sendGoal(goal);


				if(enemyDirection.compare("front")==0){
				}
				if(enemyDirection.compare("right")==0){
				}
				if(enemyDirection.compare("left")==0){
				}
				if(enemyDirection.compare("notexist")==0){
				}
			}
          		if(searchEnemyResult.compare("found")==0){
				ros::param::set("actionMode","aimEnemyTarget");
			}
		}
*/

	  }
          if(actionMode.compare("aimEnemyTarget")==0){
/*
		//敵のターゲットを探すために５秒待つ
		rate.sleep();
		rate.sleep();
		rate.sleep();

		//敵のターゲットを探した結果を確認する

      		if(ros::param::get("/aimEnemyTargetResult",aimEnemyTargetResult)){
          		if(aimEnemyTargetResult.compare("fail")==0){
                                ROS_INFO("!!!***!!! back to patrol mode (2)");
				ros::param::set("actionMode","patrol");
              		  	// サーバーにgoalを送信
              		 	if(sendGoalFlgAtAimEnemyTarget) ac.sendGoal(goal);

				if(enemyDirection.compare("front")==0){
				}
				if(enemyDirection.compare("right")==0){
				}
				if(enemyDirection.compare("left")==0){
				}
				if(enemyDirection.compare("notexist")==0){
				}
			}
          		if(aimEnemyTargetResult.compare("found")==0){
				ros::param::set("actionMode","searchEnemy");
			}
		}
*/
	  }


          if(actionMode.compare("aimFieldTarget")==0){

	  }
          if(actionMode.compare("searchPit")==0){

	  }
          if(actionMode.compare("naviRecovery")==0){

	  }
          if(actionMode.compare("moveToRightSide")==0){

	  }
          if(actionMode.compare("moveToLeftSide")==0){

	  }
          if(actionMode.compare("moveToEnemyRightSide")==0){

	  }
          if(actionMode.compare("moveToEnemyLeftSide")==0){
	  }


      }

		//pub_goalWaypointPose.publish(goalWaypointPose);
  }

  return 0;
}
