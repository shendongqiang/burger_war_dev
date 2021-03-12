#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <turtlesim/Pose.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <sstream>

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

class ScanStopper {
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
	double MIN_SCAN_ANGLE_RAD = -180.0/180*3.14;
	double MAX_SCAN_ANGLE_RAD = +180.0/180*3.14;
	//double MIN_SCAN_ANGLE_RAD = -1.998;
	//double MAX_SCAN_ANGLE_RAD = +1.998;

	double RECOVERY_MIN_SCAN_ANGLE_RAD = -90.0/180*3.14;
	double RECOVERY_MAX_SCAN_ANGLE_RAD = +90.0/180*3.14;

	double FRONT_MIN_SCAN_ANGLE_RAD = -15.0/180*3.14;
	double FRONT_MAX_SCAN_ANGLE_RAD = +15.0/180*3.14;

	// Should be smaller than sensor_msgs::LaserScan::range_max
	double  BAN_PROXIMITY_RANGE_M = 0.07;  //0.15->0.30->0.25 =>Turtlebot3 0.07
	double  MIN_PROXIMITY_RANGE_M = 0.09;  //0.25->0.35->0.30 =>Turtlebot3 0.09
	double  BACK_NAVIGATION_RANGE = 0.1;  //0.35->0.40->0.40 =>Turtlebot3 0.09

	geometry_msgs::Pose goalWaypointPose;

	ScanStopper();
	void startMoving();
	void turnAround(int direcIndicator);

    private:
	ros::NodeHandle nh;
	// Publisher to the robot's mode topic
	ros::Publisher mode_pub; 
	// Publisher to the robot's velocity command topic
	ros::Publisher cmd_pub; 
	// Subscriber to the robot's laser scan topic
	ros::Subscriber laser_sub; 
	// Subscriber to the robot's odom topic
	ros::Subscriber odom_sub; 
	  ros::Subscriber myOdom_sub; 
	  ros::Subscriber enemyOdom_sub;

	// Subscriber to the robot's goalWaypointPose topic
	ros::Subscriber pose_sub; 

	// Indicates whether the robot should continue moving
	bool keepMoving; 
	int rotationCounter;
	bool recoveryFlg;
	int recoveryCounter;

	void moveForward();
	void moveBackward();
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_);
	  void myOdomCallBack(const nav_msgs::Odometry::ConstPtr& mymsg);
	  void enemyOdomCallBack(const nav_msgs::Odometry::ConstPtr& odom_);

	void turnToTarget();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void poseCallback(const geometry_msgs::Pose::ConstPtr& pose_);

	OdomPose myPose;
	nav_msgs::Odometry myOdom;
	OdomPose enemyPose;
	nav_msgs::Odometry enemyOdom;
	//geometry_msgs::Twist vel;

};

ScanStopper::ScanStopper()
{
	//keepMoving = true;
	keepMoving = false;

	// Advertise a new publisher for the robot's mode topic
	mode_pub = nh.advertise<std_msgs::String>("mobile_base/command/mode", 10);

	// Advertise a new publisher for the simulated robot's velocity command topic
	cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	// Subscribe to the simulated robot's laser scan topic
	//laser_sub = nh.subscribe("base_scan", 1, &ScanStopper::scanCallback, this);
	laser_sub = nh.subscribe("scan", 1, &ScanStopper::scanCallback, this);
	pose_sub = nh.subscribe<geometry_msgs::Pose>("goalWaypointPose", 1, &ScanStopper::poseCallback, this);

	odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &ScanStopper::odomCallback, this);
	  myOdom_sub = nh.subscribe("odom", 1, &ScanStopper::myOdomCallBack, this);
	  enemyOdom_sub = nh.subscribe("enemy_bot/odom", 1, &ScanStopper::enemyOdomCallBack, this);

}

/**
 *  @brief ROSのトピックのクオータニオンの構造体から
 *         Roll,Pitch,Yaw角を取得する関数
 *  @param q トピックのクオータニオン
 *  @param[out] roll [rad]
 *  @param[out] pitch [rad]
 *  @param[out] yaw [rad]
 */
void GetRPY(const geometry_msgs::Quaternion &q,
	double &roll,double &pitch,double &yaw){
	//bulletのクオータニオンに変換
	//btQuaternion btq(q.x,q.y,q.z,q.w);
	//std::cout<<"--------Quaternion---------"<<q.x<<" "<<q.y<<" "<<q.z<<" "<<q.w<<std::endl;
	ROS_INFO("--------Quaternion--------- x=%5.2f  y=%5.2f  z=%5.2f  w=%5.2f " "", q.x,q.y,q.z,q.w);
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	//std::cout<<"-++++++--Yaw-++++++-"<<roll<<" "<<pitch<<" "<<yaw<<std::endl;
	ROS_INFO("-++++++--Yaw-++++++- roll=%8.3lf , pitch=%8.3lf , yaw=%8.3lf", roll,pitch,yaw);
}


// Process the incoming pose message
void ScanStopper::poseCallback(const geometry_msgs::Pose::ConstPtr& _pose) 
{
	std::string actionMode;
	std::string recoveryMode;

	if(ros::param::get("/actionMode",actionMode)){
		if(	actionMode.compare("naviRecovery")==0
			){

	if(ros::param::get("/recoveryMode",recoveryMode)){
		if(	recoveryMode.compare("turnToTarget")==0){

		goalWaypointPose=*_pose;

		}
	}
		}
	}

}


// Process the incoming odom message
// 走行に失敗したアクションゴールに向かって進む
void ScanStopper::odomCallback(const nav_msgs::Odometry::ConstPtr& _odom) 
{
	geometry_msgs::Twist command_;
	nav_msgs::Odometry odom_;
	odom_ = *_odom;
	double 	start_x_, start_y_, start_theta_; 	//turtleのの初期位置
	double 	dis_error_, theta_error_;		//目標位置への誤差
	double 	angle_, len_;				//目標への移動距離と角度

	//geometry_msgs::Quaternion q;
	//q=odom_.pose.pose.orientation;

	std::string actionMode;
	std::string recoveryMode;
	std::string originalMode;

	if(ros::param::get("/actionMode",actionMode)){
		if(actionMode.compare("naviRecovery")==0){
			if(ros::param::get("/recoveryMode",recoveryMode)){
				if(recoveryMode.compare("turnToTarget")==0){


	double roll,pitch,yaw;
	GetRPY(odom_.pose.pose.orientation,roll,pitch,yaw);

	//shen 20180829
	//geometry_msgs::Pose msg;
	turtlesim::Pose msg;
        //msg.x = odom_->pose->pose->position->x;
        //msg.y = odom_->pose->pose->position->y;
        //msg.theta = odom_->twist->twist->angular->z;
        msg.x = odom_.pose.pose.position.x;
        msg.y = odom_.pose.pose.position.y;
        //msg.theta = odom_.twist.twist.angular.z;
        //msg.theta = odom_.pose.pose.orientation;
        msg.theta = yaw;

	ROS_INFO("--------Pose--------- msgX=%5.2f  msgY=%5.2f  msgTheta=%8.3lf", msg.x,msg.y,msg.theta);

  
		//スケールパラメータを定義する
		double l_scale   = 4.0;
		double a_scale   = 4.0;
		double error_tol = 0.3; //0.01 ->0.3

		double 	l_limit_= 0.3;  //linear speed limit
		double 	a_limit_= 0.2;  //angular speed limit


			//初期座標を取得する
			start_x_ = msg.x;
			start_y_ = msg.y;
			//進行方向(x)を基準しているので、0で初期化
			start_theta_ = 0;

			//----#######----------
			//----#######----------
			//----#######----------
			//----#######----------
			
			//ウェイポイントを取得して、目標移動距離と角度を計算する
			//geometry_msgs::Pose p = goalWaypointPose;

			//----#######----------
			//----#######----------
			//----#######----------
			//----#######----------


			len_ = fabs(sqrt((goalWaypointPose.position.x - start_x_)*(goalWaypointPose.position.x - start_x_)
				       + (goalWaypointPose.position.y - start_y_)*(goalWaypointPose.position.y - start_y_)));
			angle_  = atan2((goalWaypointPose.position.y-start_y_), (goalWaypointPose.position.x-start_x_));



		ROS_DEBUG("current position:(%.3f, %.3f) theta: %.3f", msg.x, msg.y, msg.theta);
		//目標距離までの誤差を計算する
		dis_error_   = len_ - fabs(sqrt((start_x_- msg.x)*(start_x_-msg.x) 
				+ (start_y_-msg.y)*(start_y_-msg.y)));
		//目標角度までの誤差を計算する
		theta_error_ = angle_ - (msg.theta - start_theta_);
		//theta_error_ = angles::normalize_angle_positive(angle_ - (msg.theta - start_theta_));
    
		//先に回転して、次に移動する
      		if (fabs(theta_error_) > error_tol) { 
			command_.linear.x  = 0;
			command_.angular.z = a_scale*theta_error_;
			if(command_.angular.z >0 && command_.angular.z > a_limit_){command_.angular.z  =a_limit_ ;}
			if(command_.angular.z <0 && command_.angular.z < a_limit_*(-1)){command_.angular.z  =a_limit_*(-1) ;}

			//制御命令を配布する
			cmd_pub.publish(command_);

		} else {
			ros::param::set("/recoveryMode","normalRecovery");


			ROS_INFO("Obstacle has been avoided successfully, back to navigation!");
			keepMoving = false;
			if(ros::param::get("/originalActionMode",originalMode)){
				ros::param::set("/actionMode",originalMode);
			}


		} 

				}
			}
		}
	}
}

// /自分のodomトピックから自分の位置と姿勢、速度を表示
void ScanStopper::myOdomCallBack(const nav_msgs::Odometry::ConstPtr& mymsg) {
//void DrivingControl::odomCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
    myOdom = *mymsg;
    ROS_INFO("Seq: %d", myOdom.header.seq);
    ROS_INFO("/odom Pos (x:%f, y:%f, z:%f)", myOdom.pose.pose.position.x,myOdom.pose.pose.position.y, myOdom.pose.pose.position.z);
    std::string actionMode;
    std::string recoveryMode;
    //double dis=2.4; //fabs(sqrt())
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
    //len_ = fabs(sqrt((myPose.x +2.4 - enemyPose.x)*(myPose.x +2.4 - enemyPose.x)
    len_ = fabs(sqrt((myPose.x - (2.4 + enemyPose.x))*(myPose.x - (2.4 + enemyPose.x))
                   + (myPose.y - enemyPose.y)*(myPose.y - enemyPose.y)));
    //angle_  = atan2((myPose.y - enemyPose.y), (myPose.x - enemyPose.x));
    angle_  = atan2((myPose.y - enemyPose.y), (myPose.x + enemyPose.x));


     ROS_INFO("### distance form enemy :%f", len_);
     ROS_INFO("### angle with enemy :%f", angle_);


    if(len_<=1.2){
        ros::param::set("/actionDuration","shortTime");
        ros::param::set("originalActionMode","naviRecovery");
        ros::param::set("/actionMode","aimEnemy");
    }else {
        if(len_> 1.5){
            ros::param::set("/actionDuration","longTime");
        }
        if(len_> 2.0){
            ros::param::set("/actionDuration","longlongTime");
        }

	if(ros::param::get("/actionMode",actionMode)){
          if(actionMode.compare("aimEnemy")==0){
            if(len_> 1.5){
              ros::param::set("/actionMode","patrol");
            }
          }
        }
    }


}

// /敵のodomトピックから敵の位置と姿勢、速度を表示
void ScanStopper::enemyOdomCallBack(const nav_msgs::Odometry::ConstPtr& _odom) {
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


// Send a velocity command
void ScanStopper::moveForward() {
	// The default constructor will set all commands to 0
	geometry_msgs::Twist msg; 
	msg.linear.x = FORWARD_SPEED_MPS;
	msg.angular.z = 0.0;
	cmd_pub.publish(msg);
}

// Send a velocity command
void ScanStopper::moveBackward() {
	// The default constructor will set all commands to 0
	geometry_msgs::Twist msg; 
	msg.linear.x = BACKWARD_SPEED_MPS;
	msg.angular.z = 0.0;
	cmd_pub.publish(msg);
}


// Send a velocity command
void ScanStopper::turnToTarget() {
	// The default constructor will set all commands to 0

	std::string actionMode;
	std::string recoveryMode;

	ros::Rate rate(1);
	// Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
	while (ros::ok()) {


		if(ros::param::get("/actionMode",actionMode)){
			if(	actionMode.compare("naviRecovery")==0){


				if(ros::param::get("/recoveryMode",recoveryMode)){
					if(	recoveryMode.compare("turnToTarget")==0){

						// Need to call this function often to allow ROS to process incoming messages
						ros::spinOnce(); 

					}else{
						break;
					}
				}


			}
		}


		rate.sleep();
	}

}

// Send a Rotation command
void ScanStopper::turnAround(int direcIndicator) {
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

// Send a Rotation command
/*
void ScanStopper::turnAround() {
	// The default constructor will set all commands to 0
	double rotationSpeed = ROTATION_SPEED_PIPS;

	geometry_msgs::Twist msg; 
	msg.linear.x = 0.0;
	msg.angular.z = rotationSpeed;
	cmd_pub.publish(msg);
}
*/

// Process the incoming laser scan message
void ScanStopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	std::string actionMode;
	std::string recoveryMode;
	std::string odomMode;
        double frontAngle;

	// Find the closest range between the defined minimum and maximum angles
        //
        //frame_id: "base_scan"
        //angle_min: 0.0
        //angle_max: 6.28318977356
        //angle_increment: 0.0175019223243
        //range_min: 0.119999997318
        //range_max: 3.5

    //ROS_INFO("###+++### scan info ###+++###");
    //ROS_INFO("range_min :%f) ", scan->range_min);
    //ROS_INFO("range_max :%f) ", scan->range_max);
    //ROS_INFO("angle_min :%f) ", scan->angle_min);
    //ROS_INFO("angle_max :%f) ", scan->angle_max);

  int i = scan->ranges.size() / 2;
  if (scan->ranges[i] < scan->range_min || // エラー値の場合
      scan->ranges[i] > scan->range_max || // 測定範囲外の場合
      std::isnan(scan->ranges[i])) // 無限遠の場合
  {
    //ROS_INFO("front-range: measurement error");
  }
  else
  {
    ROS_INFO("###+++### front-range: %0.3f",
      scan->ranges[scan->ranges.size() / 2]);

    frontAngle = (scan->angle_max - scan->angle_min) / 2;
    //ROS_INFO("###+++### frontAngle: %0.3f", frontAngle);

    int frontIndex;
    frontIndex = ceil(frontAngle / scan->angle_increment);
    //ROS_INFO("###+++### front-index: %i", frontIndex);
  }




	if(ros::param::get("/actionMode",actionMode)){
		if(actionMode.compare("naviRecovery")==0){

	if(ros::param::get("/recoveryMode",recoveryMode)){
		if(	recoveryMode.compare("normalRecovery")==0){

	//normalMode制御命令を配布する
	std_msgs::String modeString;
	modeString.data="normal";
	mode_pub.publish(modeString);

	int minIndex =  ceil(scan->angle_min / scan->angle_increment);
	int maxIndex = floor(scan->angle_max / scan->angle_increment);
	//int minIndex =  ceil((frontAngle + RECOVERY_MIN_SCAN_ANGLE_RAD) / scan->angle_increment);
	//int maxIndex = floor((frontAngle + RECOVERY_MAX_SCAN_ANGLE_RAD) / scan->angle_increment);
	//int minIndex =  ceil((RECOVERY_MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	//int maxIndex = floor((RECOVERY_MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	int middleIndex = minIndex+floor((maxIndex - minIndex) / 2);
	int direcIndicator = 0;


	//std::string actionMode;
	std::string originalMode;

	//float closestRange = scan->ranges[minIndex];
	double closestRange = scan->ranges[minIndex];
	int closestIndex = minIndex;
	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
		if ((scan->ranges[currIndex] >=0.001)&&(scan->ranges[currIndex] < closestRange)) {
			closestRange = scan->ranges[currIndex];
			closestIndex = currIndex;
		}
	}
	//direcIndicator=closestIndex-middleIndex;
	direcIndicator=middleIndex - closestIndex;

	int minIndex_allDirections =  ceil((frontAngle + MIN_SCAN_ANGLE_RAD) / scan->angle_increment);
	int maxIndex_allDirections = floor((frontAngle + MAX_SCAN_ANGLE_RAD) / scan->angle_increment);
	//int minIndex_allDirections =  ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	//int maxIndex_allDirections = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	double closestRange_allDirections = scan->ranges[minIndex_allDirections];
	int closestIndex_allDirections = minIndex_allDirections;
	for (int currIndex = minIndex_allDirections + 1; currIndex <= maxIndex_allDirections; currIndex++) {
		if ((scan->ranges[currIndex] >=0.001)&&(scan->ranges[currIndex] < closestRange_allDirections)) {
			closestRange_allDirections = scan->ranges[currIndex];
			closestIndex_allDirections = currIndex;
		}
	}

	int minIndex_frontDirections =  ceil((frontAngle + FRONT_MIN_SCAN_ANGLE_RAD) / scan->angle_increment);
	int maxIndex_frontDirections = floor((frontAngle + FRONT_MAX_SCAN_ANGLE_RAD) / scan->angle_increment);
	//int minIndex_frontDirections =  ceil((FRONT_MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	//int maxIndex_frontDirections = floor((FRONT_MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	double closestRange_frontDirections = scan->ranges[minIndex_frontDirections];
	int closestIndex_frontDirections = minIndex_frontDirections;
	for (int currIndex = minIndex_frontDirections + 1; currIndex <= maxIndex_frontDirections; currIndex++) {

		ROS_INFO_STREAM("---Closest range_frontDirections---: " << closestRange_frontDirections);
		ROS_INFO_STREAM("+++Closest Index_frontDirections+++: " << closestIndex_frontDirections);

		if ((scan->ranges[currIndex] >=0.001)&&(scan->ranges[currIndex] < closestRange_frontDirections)) {
			closestRange_frontDirections = scan->ranges[currIndex];
			closestIndex_frontDirections = currIndex;
		}
	}

	ROS_INFO_STREAM("*** minIndex: " << minIndex);
	ROS_INFO_STREAM("*** maxIndex: " << maxIndex);
	ROS_INFO_STREAM("*** middleIndex: " << middleIndex);

	ROS_INFO_STREAM("Closest range: " << closestRange);
	ROS_INFO_STREAM("Closest index: " << closestIndex);
	ROS_INFO_STREAM("turn direction: " << direcIndicator);
	//ROS_INFO_STREAM("Closest range_frontDirections: " << closestRange_frontDirections);
	ROS_INFO_STREAM("Closest range_recovery: " << closestRange);
	//ROS_INFO_STREAM("Closest range_allDirections: " << closestRange_allDirections);

	if (closestRange_frontDirections < BAN_PROXIMITY_RANGE_M) {
		rotationCounter++;
		recoveryFlg=true;
		recoveryCounter=0;

		ROS_INFO("Obstacle discovery! moveBackward avoiding!");
		keepMoving = false;
		moveBackward();

	}else if (closestRange < MIN_PROXIMITY_RANGE_M) {
		recoveryFlg=true;
		recoveryCounter=0;

		if(rotationCounter>120){
			ROS_INFO("Obstacle discovery! turn 90 deg to escape from the corner!");
			ros::Rate rate(1);
			//for(int i=1;i<16;i++){
			for(int i=1;i<6;i++){
				turnAround(direcIndicator);
				rate.sleep();
			}
			rotationCounter=0;
		}else{
			rotationCounter++;
			ROS_INFO("Obstacle discovery! turnAround avoiding!");
			keepMoving = false;
			turnAround(direcIndicator);
			//moveBackward();
		}

	}else if(closestRange_allDirections > BACK_NAVIGATION_RANGE) {
		rotationCounter=0;
		ros::Rate rate(1);

		if(recoveryFlg){
			recoveryCounter++;

		if(ros::param::get("/odomMode",odomMode)){
			if(odomMode.compare("enable")==0
			){
				ROS_INFO("Obstacle has been avoided successfully, turn to the target!");
				ros::param::set("/recoveryMode","turnToTarget");
				turnToTarget();	
			}else{
				ROS_INFO("Obstacle has been avoided successfully, turn 15 deg! -1-");
				//for(int j=1;j<5;j++){
				for(int j=1;j<1;j++){
					turnAround(direcIndicator);
					rate.sleep();
				}
			}
			if(recoveryCounter>1) recoveryFlg=false;
		}else{
			ROS_INFO("Obstacle has been avoided successfully, turn 15 deg! -2-");
			//for(int j=1;j<5;j++){
			for(int j=1;j<1;j++){
				turnAround(direcIndicator);
				rate.sleep();
			}
			if(recoveryCounter>1) recoveryFlg=false;
		}


		}else{
			if(recoveryCounter>4){
				ROS_INFO("Obstacle discovery! turn 370 deg to clear the cost map!");
				//for(int k=1;k<61;k++){
				for(int k=1;k<24;k++){
					turnAround(direcIndicator);
					rate.sleep();
				}
				recoveryFlg=true;
				recoveryCounter=0;
			}else{
				recoveryCounter++;

				ROS_INFO("moveForward to clear the cost map!");
				keepMoving = true;
				moveForward();
			}
		}

		ROS_INFO("Obstacle has been avoided successfully, back to navigation!");
		keepMoving = false;
		if(ros::param::get("/originalActionMode",originalMode)){
			ros::param::set("/actionMode",originalMode);
		}


/*
		ROS_INFO("Obstacle has been avoided successfully, turn to the target!");
		ros::param::set("/recoveryMode","turnToTarget");
*/
/*
		//turnToTarget();
		ros::Rate rate(1);
		// Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
		while (ros::ok()) {

			if(ros::param::get("/recoveryMode",recoveryMode)){
				if(	recoveryMode.compare("turnToTarget")==0){

					// Need to call this function often to allow ROS to process incoming messages
					ros::spinOnce(); 

				}else{
					break;
				}
			}
			rate.sleep();
		}

		ROS_INFO("Obstacle has been avoided successfully, back to navigation!");
		keepMoving = false;
		if(ros::param::get("/originalActionMode",originalMode)){
			ros::param::set("/actionMode",originalMode);
		}
*/

	}else{
		rotationCounter++;
		recoveryFlg=true;
		recoveryCounter=0;

		ROS_INFO("Obstacle discovery! moveForward avoiding!");
		keepMoving = true;
		moveForward();
	}






		}
	}


		}
	}

}

void ScanStopper::startMoving()
{
	//std::string actionMode;
	ros::Rate rate(1);
	ROS_INFO("Start moving");

	// Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
	//while (ros::ok() && keepMoving) {

	ros::param::set("/recoveryMode","normalRecovery");
	recoveryFlg=true;
	recoveryCounter=0;

	while (ros::ok()) {
/*
		//ros::param::set("/recoveryMode","normalRecovery");
		if(ros::param::get("/actionMode",actionMode)){
			if(	actionMode.compare("naviRecovery")==0){
				if(keepMoving){
					moveForward();
				}
				// Need to call this function often to allow ROS to process incoming messages
				//ros::spinOnce(); 
			}
		}
*/
		ros::spinOnce(); 
		rate.sleep();
	}
}

int main(int argc, char **argv) 
{
	// Initiate new ROS node named "stopper"
	//ros::init(argc, argv, "stopper");
	//ros::init(argc, argv, "rulo_laserAutoDrive");
	ros::init(argc, argv, "navi_recovery");

	// Create new stopper object
	ScanStopper stopper;

	// Start the movement
	stopper.startMoving();

	return 0;
}

