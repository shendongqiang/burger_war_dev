#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <iostream>
#include <string>
#include <fstream>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <turtlesim/Pose.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <sstream>


using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct MyPose {
  int num;
  double px;
  double py;
  double pz;
  double ox;
  double oy;
  double oz;
  double ow;
};

class DrivingControl {
    public:
	// Tunable parameters
	double FORWARD_SPEED_MPS = 0.2;   //0.05 ->0.2
	double BACKWARD_SPEED_MPS = -0.2; //-0.01 ->-0.1
	double ROTATION_SPEED_PIPS = 0.5; //0.5->0.3
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

	DrivingControl();
	//Patrol_cource();
	//void startMoving();
	void turnAround(int direcIndicator);
	void moveForward();
	void moveBackward();

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
	//void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_);
	//void poseCallback(const geometry_msgs::Pose::ConstPtr& pose_);
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
	//odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &ScanStopper::odomCallback, this);
	//pose_sub = nh.subscribe<geometry_msgs::Pose>("goalWaypointPose", 1, &ScanStopper::poseCallback, this);

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

int main(int argc, char** argv){

  //MyPose way_point[] = {{-0.996, 1.496, 0.898,0.440},{-0.429, 2.208, 0.027,1.000},{-1.516, 2.128, -0.719,0.695},{0.028, 0.017, -0.715,0.699}};

  ros::Publisher  pub_goalWaypointPose;

  ros::init(argc, argv, "patrol_cource");

  ros::NodeHandle nh;
  //pub_goalWaypointPose= nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("goalWaypointPose", 10, true);
  pub_goalWaypointPose= nh.advertise<geometry_msgs::Pose>("goalWaypointPose", 10, true);


	// Create new drivingControl object
	DrivingControl drivingControl;
	//Patrol_cource patrol_cource;

  // チェックポイントをway_point[]に入力していく
  std::string line="";
  float px=0.0, py=0.0, pz=0.0, ox=0.0, oy=0.0, oz=0.0, ow=0.0;
  int i=0;
  //ifstream ifs{};

  //======== get waypoints for patrol ==============

  //open patrol_waypoints file
  ifstream ifsForPatrol("/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsForPatrol.csv");
  //ifs("/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsForPatrol.csv");
  if(ifsForPatrol.fail()){
    ROS_INFO( "File do not exist.");
    exit(0);
  }

  //create wayPoints array
  MyPose waypointsForPatrol[50];

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

  //======== get waypoints To Empty Can ==============
  //open waypoints csv file for moving to EmptyCan
  ifstream ifsToEmptyCan("/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToEmptyCan.csv");
  //ifsToEmptyCan("/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToEmptyCan.csv");
  if(ifsToEmptyCan.fail()){
    ROS_INFO( "File do not exist.");
    exit(0);
  }

  //create wayPoints array
  MyPose waypointsToEmptyCan[50];

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
  ifstream ifsToGarbageBox("/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToGarbageBox.csv");
  //ifsToGarbageBox("/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToGarbageBox.csv");
  if(ifsToGarbageBox.fail()){
    ROS_INFO( "File do not exist.");
    exit(0);
  }

  //create wayPoints array
  MyPose waypointsToGarbageBox[50];


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
  ifstream ifsToDockStation("/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToDockStation.csv");
  //ifs("/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToDockStation.csv");
  if(ifsToDockStation.fail()){
    ROS_INFO( "File do not exist.");
    exit(0);
  }

  //create wayPoints array
  MyPose waypointsToDockStation[50];

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
  std::string patrolSubMode;
  std::string patrolResult;
	geometry_msgs::Pose goalWaypointPose;

  //int waypointIndex = 0;
  waypointIndexPatrol = 0;
  goalWayPointIndex=waypointIndexPatrol;
  waypointIndexEmptyCan=0;
  waypointIndexGarbageBox=0;
  waypointIndexDockStation=0;
  bool goalCancelFlg=false;
  int direcIndicator = 0;

  ros::Rate rate(1);
/*
			  //右へ回転し、右側のフィールドターゲットを探して点数を取得する
			for(int i=1;i<3;i++){
				ROS_INFO("--------waypoint(1)--------- ");
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			}

			  //左へ回転し、左側のフィールドターゲットを探して点数を取得する
			for(int i=1;i<3;i++){
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
      if(ros::param::get("/actionMode",actionMode)){

          if(actionMode.compare("patrol")==0){
              if(goalCancelFlg) goalCancelFlg=false;
		ros::param::get("/patrolSubMode",patrolSubMode);
              // ROSではロボットの進行方向がx座標、左方向がy座標、上方向がz座標
              goal.target_pose.pose.position.x =  waypointsForPatrol[waypointIndexPatrol].px;
              goal.target_pose.pose.position.y =  waypointsForPatrol[waypointIndexPatrol].py;
              goal.target_pose.pose.orientation.z = waypointsForPatrol[waypointIndexPatrol].oz; 
              goal.target_pose.pose.orientation.w = waypointsForPatrol[waypointIndexPatrol].ow; 

	      goalWaypointPose=goal.target_pose.pose;

              ROS_INFO("Sending goal: No.%d", waypointIndexPatrol);

//2021 for roboticsHubChanllenge
              // サーバーにgoalを送信
              //ac.sendGoal(goal);

		//if (waypointIndexPatrol>=patrolWaypointsNum) waypointIndexPatrol=0;

		if(waypointIndexPatrol==0){
			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する
			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");
			}

			  // waypointIndexインクリメント
			  waypointIndexPatrol++;
              		  // サーバーにgoalを送信し、次のwaypointへ走行させる
              		  ac.sendGoal(goal);
		}else if(waypointIndexPatrol==1){
		  //１つ目のWaypointに到着
			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する

			  //フィールドターゲットを探して点数を取得する
				//ros::param::set("actionMode","aimTarget");
			  //真正面のフィールドターゲットを探して点数を取得する
			  //右へ回転し、右側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<4;i++){
				ROS_INFO("--------waypoint(1)--------- ");
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }

			  //左へ回転し、左側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<5;i++){
				direcIndicator = -10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }
			  //右へ回転し、２つ目のwaypointへ走行しやすいように姿勢を調整する
			  for(int i=1;i<3;i++){
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }

			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");
			}

			  // waypointIndexインクリメント
			  waypointIndexPatrol++;
              		  // サーバーにgoalを送信し、次のwaypointへ走行させる
              		  ac.sendGoal(goal);
		}else if(waypointIndexPatrol==2){
			//２つ目のWaypointに到着
			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する
			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");
			}

			  // waypointIndexインクリメント
			  waypointIndexPatrol++;
              		  // サーバーにgoalを送信し、次のwaypointへ走行させる
              		  ac.sendGoal(goal);

		}else if(waypointIndexPatrol==3){
			//３つ目のWaypointに到着
			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する
			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");
			}

			  // waypointIndexインクリメント
			  waypointIndexPatrol++;
              		  // サーバーにgoalを送信し、次のwaypointへ走行させる
              		  ac.sendGoal(goal);

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
			  //右へ回転し、右側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<4;i++){
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }

			  //右へ回転し、右側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<4;i++){
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }

			  //左へ回転し、５つ目のwaypointへ走行しやすいように姿勢を調整する
				//ros::param::set("actionMode","aimTarget");
			  for(int i=1;i<4;i++){
				direcIndicator = -10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }
			  //後ろへバックし、５つ目のwaypointへ走行しやすいように姿勢を調整する
			  //for(int i=1;i<2;i++){
			  //	drivingControl.moveBackward();
			  //	rate.sleep();
			  //}
			  //前へ進み、５つ目のwaypointへ走行しやすいように姿勢を調整する
			  //for(int i=1;i<2;i++){
			  //	drivingControl.moveForward();
			  //	rate.sleep();
			  //}


			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");
			}

			  // waypointIndexインクリメント
			  waypointIndexPatrol++;
              		  // サーバーにgoalを送信し、次のwaypointへ走行させる
              		  ac.sendGoal(goal);

		}else if(waypointIndexPatrol==5){
			//５つ目のWaypointに到着
			if(patrolSubMode.compare("preAdjustment")==0){
			  //事前処理
			  //ターゲット捜索し、点数を取得する
			  //次のwaypointへ走行しやすいように姿勢を調整する

			  //左へ回転し、６つ目のwaypointへ走行しやすいように姿勢を調整する
				//ros::param::set("actionMode","aimTarget");
			  for(int i=1;i<4;i++){
				direcIndicator = -10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }

			  //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");
			}

			  // waypointIndexインクリメント
			  waypointIndexPatrol++;
              		  // サーバーにgoalを送信し、次のwaypointへ走行させる
              		  ac.sendGoal(goal);

		}else if(waypointIndexPatrol==6){
			//６つ目のWaypointに到着
			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する
			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");
			}

			  // waypointIndexインクリメント
			  waypointIndexPatrol++;
              		  // サーバーにgoalを送信し、次のwaypointへ走行させる
              		  ac.sendGoal(goal);

		}else if(waypointIndexPatrol==7){
			//７つ目のWaypointに到着
			if(patrolSubMode.compare("preAdjustment")==0){
			 //事前処理
			 //ターゲット捜索し、点数を取得する
			 //次のwaypointへ走行しやすいように姿勢を調整する


			  //フィールドターゲットを探して点数を取得する
				//ros::param::set("actionMode","aimTarget");
			  //左へ回転し、左側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<4;i++){
				direcIndicator = -10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }
			  //左へ回転し、左側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<4;i++){
				direcIndicator = -10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }
			  //左へ回転し、左側のフィールドターゲットを探して点数を取得する
			  for(int i=1;i<4;i++){
				direcIndicator = -10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }
			  //右へ回転し、８つ目のwaypointへ走行しやすいように姿勢を調整する
			  for(int i=1;i<8;i++){
				direcIndicator = 10;
				drivingControl.turnAround(direcIndicator);
				//patrol_cource.turnAround(direcIndicator);
				rate.sleep();
			  }
			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","movingToGoal");
			}

			  // waypointIndexインクリメント
			  waypointIndexPatrol++;
              		  // サーバーにgoalを送信し、次のwaypointへ走行させる
              		  ac.sendGoal(goal);

		//}else if (waypointIndexPatrol>=patrolWaypointsNum) {
		}else if (waypointIndexPatrol>=8) {
			//８つ目のWaypointに到着
			  if(patrolSubMode.compare("preAdjustment")==0){
			  //少し前に出て、敵を探しやすいように姿勢を調整する
			  for(int i=1;i<3;i++){
				drivingControl.moveForward();
				rate.sleep();
			  }
			  //左へ少し回転し、敵を探しやすいように姿勢を調整する
			  for(int i=1;i<2;i++){
				direcIndicator = -10;
				drivingControl.turnAround(direcIndicator);
				rate.sleep();
			  }
			  //敵に向かわせせるようにして点数を取得する
			  ros::param::set("actionMode","aimEnemy");

			  //サブ走行モード設定：patrolSubMode
			  ros::param::set("patrolSubMode","movingToGoal");
			}

			  //最後のwaypointでは、点数を取得したあとに、試合終了まで、そのまま待機する
			  //ros::param::set("actionMode","standBy");
			  // waypointIndexを初期化
			  //waypointIndexPatrol=0;
              		  // サーバーにgoalを送信しない
              		  //ac.sendGoal(goal);
		}else {

		}

              	// 結果が返ってくるまで30.0[s] 待つ。ここでブロックされる。
              	//bool succeeded = ac.waitForResult(ros::Duration(120.0));
              	//ac.waitForResult(ros::Duration(30.0));
              	ac.waitForResult(ros::Duration(20.0));

              	// 結果を見て、成功ならSucceeded、失敗ならFailedと表示
              	actionlib::SimpleClientGoalState state = ac.getState();

		patrolResult=state.toString().c_str();
          	if(patrolResult.compare("SUCCEEDED")==0){
	       		ROS_INFO("Succeeded: waypointIndexPatrol No.%d (%s)",goalWayPointIndex, state.toString().c_str());
			goalWayPointIndex=waypointIndexPatrol;
               		ROS_INFO("nextWayPointIndex: No.%d",goalWayPointIndex);
			 //サブ走行モード設定：patrolSubMode
			   ros::param::set("patrolSubMode","preAdjustment");

		}else {
			pub_goalWaypointPose.publish(goalWaypointPose);

                	ROS_INFO("Failed: waypointIndexPatrol No.%d (%s)",(waypointIndexPatrol-1), state.toString().c_str());
			//サブ走行モード設定(patrolSubMode)を変更しない：movingToGoalのまま
			//ros::param::set("patrolSubMode","movingToGoal");

			//次のwaypointへ進まないようにする
			waypointIndexPatrol--;
			goalWayPointIndex=waypointIndexPatrol;
               		//ROS_INFO("nextWayPointIndex: No.%d",goalWayPointIndex);

			//アクション失敗時に、リカバリ走行させる
			//ros::param::set("originalActionMode","patrol");
			//ros::param::set("actionMode","naviRecovery");
            	}
	  } else{
             //abort
              if(!goalCancelFlg){
                //ac.cancelGoal();
                //goalCancelFlg=true;
  	        //waypointIndexPatrol = 0;
              }
          }

          if(actionMode.compare("moveBackHeadquarter")==0){

	  }

          if(actionMode.compare("moveToEnemyHeadquarter")==0){

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
