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

using namespace std;

double errAOld=0;
double errDOld=0;

//Publisher
ros::Publisher pubCmdVel;

//For turtlesim
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
ros::Publisher velocity_publisher;

// Variable
geometry_msgs::Twist twistVitesse;

const double PI = 3.14159265359;

//Fonctions
void move(double speed, double distance, bool isForward);
void rotate (double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void setDesiredOrientation (double desired_angle_radians);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void moveGoal(turtlesim::Pose  goal_pose, double distance_tolerance, double t0);
double normalizeRotation(double angle_in_degrees);

int main(int argc, char** argv){

    ros::init(argc, argv, "suivi_chemin");
	ros::NodeHandle noeud_suivi; // declaration noeud
	
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;
	
	//Pour turtlesim
	velocity_publisher = noeud_suivi.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = noeud_suivi.subscribe("/turtle1/pose", 10, poseCallback);
	

	robmobile_projet::Tab_point::ConstPtr chemin_message = ros::topic::waitForMessage<robmobile_projet::Tab_point>("/tab_chemin");
	
	ROS_INFO("On a recu un tableau, premier point: X(1)= %f, Y(1) = %f, size tab= %d\n",chemin_message->tab_points_X[1],chemin_message->tab_points_Y[1],chemin_message->size);
	ROS_INFO("On a recu un tableau, dernier point : X(fin)= %f, Y(fin) = %f, size tab= %d\n",chemin_message->tab_points_X[chemin_message->size-1],chemin_message->tab_points_Y[chemin_message->size-1],chemin_message->size);
	
	ROS_INFO("\n\n\n******START TESTING************\n");

	turtlesim::Pose pose;

	for(int i=0;i<chemin_message->size;i++){
		double t0 = ros::Time::now().toSec();
		pose.x=chemin_message->tab_points_X[i]/5;
		pose.y=chemin_message->tab_points_Y[i]/5;
		pose.theta=0;
		moveGoal(pose,0.01,t0);
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
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);

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
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle<relative_angle);

	vel_msg.angular.z =0;
	velocity_publisher.publish(vel_msg);

}

double normalizeRotation(double angle_in_degrees){
	double angle_normalized;
	angle_normalized=(double)((int)angle_in_degrees%360);
	if(angle_normalized<0){
		angle_normalized+=360;
	}
	return angle_normalized;
}

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}


void setDesiredOrientation (double desired_angle_radians){
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians<0)?true:false);
	rotate (degrees2radians(10), abs(relative_angle_radians), clockwise);

}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}


double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}


void moveGoal(turtlesim::Pose  goal_pose, double distance_tolerance, double t0){

	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(100);
	
	printf("Je rentre bien dans cette fonction\n");
	double t1 = ros::Time::now().toSec();
	double deltaT=t1-t0;
	double errDOld=0;
	double errAngleOld=0;
	do{
		/****** PID sur la distance******/

		double Kp=1.0;
		double Ki=1.0;
		double Kd=1.0;
		
		double errD=getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		double errDIntegrale=0;
		
		errDIntegrale+=errD*deltaT;
		if(errDIntegrale > 10){
			errDIntegrale=10;
		}
		
		if(errDIntegrale < -10){
			errDIntegrale=-10;
		}
		
		double errDDeriv;
		errDDeriv=(errD-errDOld)/deltaT;
		errDOld=errD;

		double vit=Kp*errD+Ki*errDIntegrale+Kd*errDDeriv;
		
		/****** PID sur l'angle******/
		double KpA=4.0;
		double KiA=1.0;
		double KdA=1.0;
		
		double errA=(atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x)-turtlesim_pose.theta);
		double errAnormalise=normalizeRotation(errA);
		double errAIntegrale=0;
		
		errAIntegrale+=errAnormalise*deltaT;
		if(errAIntegrale > 5){
			errAIntegrale=5;
		}
		
		if(errAIntegrale < -5){
			errAIntegrale=-5;
		}
		
		double errADeriv;
		errADeriv=(errAnormalise-errAOld)/deltaT;
		errAOld=errAnormalise;
		double vitAngle=KpA*errA+KiA*errAIntegrale+KdA*errADeriv;
		
		
		vel_msg.linear.x = (vit);
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =(vitAngle);

		velocity_publisher.publish(vel_msg);
		
		/*u1 =vit;
		theta_e=errA
		u2 = -u1*sin(theta_e) /(l1 * cos(theta_e)) - u1 * K * distance / (cos(theta_e));
		commande_vitesse.linear.x = u1;
		commande_vitesse.angular.z = u2;*/
  
		ros::spinOnce();
		loop_rate.sleep();

	}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);
        vel_msg.linear.x =0;
        vel_msg.angular.z = 0;
        velocity_publisher.publish(vel_msg);
}




