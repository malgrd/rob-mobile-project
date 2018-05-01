#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include "turtlesim/Pose.h"
#include <sstream>
#include "robmobile_projet/Tab_point.h"
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>

using namespace std;


//Publisher
ros::Publisher pubCmdVel;

//Subscriber
//ros::Subscriber sub_tab_points;
ros::Subscriber sub_odom;

//For turtlesim
//ros::Subscriber pose_subscriber;
//turtlesim::Pose turtlesim_pose;
//ros::Publisher velocity_publisher;

// Variable
geometry_msgs::Twist twistVitesse;

nav_msgs::Odometry odom;
double angle_robot;

const double PI = 3.14159265359;

//Fonctions
void move(double speed, double distance, bool isForward);
void rotate (double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void setDesiredOrientation (double desired_angle_radians);
//void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void poseOdomCallback(const nav_msgs::Odometry::ConstPtr & pose_odom_message);
//void moveGoal(turtlesim::Pose  goal_pose, double distance_tolerance);
void moveGoal(geometry_msgs::PoseStamped  goal_pose, double distance_tolerance);


int main(int argc, char** argv){

    ros::init(argc, argv, "suivi_chemin_gazebo");
	ros::NodeHandle noeud_suivi; // declaration noeud
	
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;
	
	//Pour turtlesim
	//velocity_publisher = noeud_suivi.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	//pose_subscriber = noeud_suivi.subscribe("/turtle1/pose", 10, poseCallback);
	
	//Sans turtlesim
	sub_odom=noeud_suivi.subscribe<nav_msgs::Odometry>("/odom",100, poseOdomCallback);
	pubCmdVel = noeud_suivi.advertise<geometry_msgs::Twist>("/cmd_vel",10);// creation d'un publisher pour le topic cmd_vel

	robmobile_projet::Tab_point::ConstPtr chemin_message = ros::topic::waitForMessage<robmobile_projet::Tab_point>("/tab_chemin");
	
	ROS_INFO("On a recu un tableau, premier point: X(1)= %f, Y(1) = %f, size tab= %d\n",chemin_message->tab_points_X[1],chemin_message->tab_points_Y[1],chemin_message->size);
	ROS_INFO("On a recu un tableau, dernier point : X(fin)= %f, Y(fin) = %f, size tab= %d\n",chemin_message->tab_points_X[chemin_message->size-1],chemin_message->tab_points_Y[chemin_message->size-1],chemin_message->size);
	
	ROS_INFO("\n\n\n******START TESTING************\n");

	//turtlesim::Pose pose;
	geometry_msgs::PoseStamped pose_robot;

	for(int i=0;i<chemin_message->size;i++){
		pose_robot.pose.position.x=chemin_message->tab_points_X[i];
		pose_robot.pose.position.y=chemin_message->tab_points_Y[i];
		moveGoal(pose_robot,0.01);
		printf("Tableau des x : %f\n", chemin_message->tab_points_X[i]);
		printf("Tableau des y : %f\n", chemin_message->tab_points_Y[i]);
	}

	ros::spin();
	return 0;
}

/**
 *  makes the robot move with a certain linear velocity for a
 *  certain distance in a forward or backward straight direction.
 */
void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do{
		pubCmdVel.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);
	vel_msg.linear.x =0;
	pubCmdVel.publish(vel_msg);

}


void rotate (double angular_speed, double relative_angle, bool clockwise){

	geometry_msgs::Twist vel_msg;

	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise)
		vel_msg.angular.z =-abs(angular_speed);
	else
		vel_msg.angular.z =abs(angular_speed);

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);
	do{
		pubCmdVel.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle<relative_angle);

	vel_msg.angular.z =0;
	pubCmdVel.publish(vel_msg);

}

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}


void setDesiredOrientation (double desired_angle_radians){
	//double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	double relative_angle_radians = desired_angle_radians - angle_robot;
	bool clockwise = ((relative_angle_radians<0)?true:false);
	rotate (degrees2radians(10), abs(relative_angle_radians), clockwise);

}

/*void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}*/

void poseOdomCallback(const nav_msgs::Odometry::ConstPtr & pose_odom_message){
	odom.pose.pose.position.x=pose_odom_message->pose.pose.position.x;
	odom.pose.pose.position.y=pose_odom_message->pose.pose.position.y;
	odom.pose.pose.orientation=pose_odom_message->pose.pose.orientation;
	
	double roll, pitch, yaw, angle_robot;
	tf::Quaternion quater;
	tf::quaternionMsgToTF(pose_odom_message->pose.pose.orientation,quater);
	tf::Matrix3x3(quater).getRPY(roll,pitch,yaw);
	
	yaw= angles::normalize_angle_positive(yaw);//rotation en z
	angle_robot=yaw;
	
}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void moveGoal(geometry_msgs::PoseStamped  goal_pose, double distance_tolerance){

	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(100);
	do{
		/****** Proportional Controller ******/

		double Kp=0.5;

		vel_msg.linear.x = (Kp*getDistance(odom.pose.pose.position.x, odom.pose.pose.position.y, goal_pose.pose.position.x, goal_pose.pose.position.y));
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =4*(atan2(goal_pose.pose.position.y-odom.pose.pose.position.y, goal_pose.pose.position.x-odom.pose.pose.position.x)-angle_robot);

		pubCmdVel.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}while(getDistance(odom.pose.pose.position.x, odom.pose.pose.position.y, goal_pose.pose.position.x, goal_pose.pose.position.y)>distance_tolerance);
        vel_msg.linear.x =0;
        vel_msg.angular.z = 0;
		pubCmdVel.publish(vel_msg);
}




