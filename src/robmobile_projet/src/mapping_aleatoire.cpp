#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <time.h>

// On controle la vitesse du robot
ros::Publisher pub;

// Valeur des capteurs
ros::Subscriber laser1;
ros::Subscriber laser2;
ros::Subscriber laser3;
ros::Subscriber laser4;
ros::Subscriber laser5;

// Variable
geometry_msgs::Twist twist;
double PI = 3.141592;
bool Laser[5] = {false};
double LaserPortee = 0.6;// Scope of the laser

//Fonctions

void Move(geometry_msgs::Twist& twist);
void capt1(sensor_msgs::LaserScan data);
void capt2(sensor_msgs::LaserScan data);
void capt3(sensor_msgs::LaserScan data);
void capt4(sensor_msgs::LaserScan data);
void capt5(sensor_msgs::LaserScan data);

int main(int argc, char** argv){

    ros::init(argc, argv, "mapping_aleatoire");
	ros::NodeHandle noeud; // declaration noeud
	ros::Time t;
	t = ros::Time::now();

	// publish suscribe pour vitesses et lasers
	pub = noeud.advertise<geometry_msgs::Twist>("/cmd_vel",10);// creation d'un publisher pour le topic cmd_vel
	laser1 = noeud.subscribe<sensor_msgs::LaserScan>("/IR1", 1000, capt1);
	laser2 = noeud.subscribe<sensor_msgs::LaserScan>("/IR2", 1000, capt2);
	laser3 = noeud.subscribe<sensor_msgs::LaserScan>("/IR3", 1000, capt3);
    laser4 = noeud.subscribe<sensor_msgs::LaserScan>("/IR4", 1000, capt4);
	laser5 = noeud.subscribe<sensor_msgs::LaserScan>("/IR5", 1000, capt5);

	ros::Rate loop_rate(100);

	while(ros::ok()){

        Move(twist);
        pub.publish(twist);
		ros::spinOnce();
		loop_rate.sleep();
	}	
	return 0;
	
}

void Move(geometry_msgs::Twist& twist){
        
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;

	//std::cout << Laser[0] << " " << Laser[1] << " " << Laser[2] << " " << Laser[3] << " " << Laser[4] << std::endl;
	if(Laser[0]==false && Laser[1] == false && Laser[2] == false && Laser[3] == false && Laser[4] == false){
		twist.linear.x = 0.5;
	}
	else if( (Laser[0] == true || Laser[1] ==true) && (Laser[3] == false && Laser[4] == false)){
		twist.angular.z = PI/6;
	}
	else if( (Laser[3] == true || Laser[4] ==true) && (Laser[1] == false && Laser[0] == false)){
		twist.angular.z = PI/6;
	}
	else{
		twist.angular.z = PI/3;
	}
}


void capt1(sensor_msgs::LaserScan data){
	float intensity = data.ranges[0];
	if(intensity < LaserPortee- 0.01){
		Laser[0] = true;
	}else{
		Laser[0] = false;
	}
}

void capt2(sensor_msgs::LaserScan data){
	float intensity = data.ranges[0];
	if(intensity < LaserPortee- 0.01){
		Laser[1] = true;
	}else{
		Laser[1] = false;
	}
}

void capt3(sensor_msgs::LaserScan data){
	//std::cout << "laser 3 mise à jour" << std::endl;
	float intensity = data.ranges[0];
	if(intensity < LaserPortee- 0.01){
		Laser[2] = true;
	}else{
		Laser[2] = false;
	}
}

void capt4(sensor_msgs::LaserScan data){
	float intensity = data.ranges[0];
	if(intensity < LaserPortee- 0.01){
		Laser[3] = true;
	}else{
		Laser[3] = false;
	}
}

void capt5(sensor_msgs::LaserScan data){
	float intensity = data.ranges[0];
	if(intensity < LaserPortee- 0.01){
		Laser[4] = true;
	}else{
		Laser[4] = false;
	}
}
