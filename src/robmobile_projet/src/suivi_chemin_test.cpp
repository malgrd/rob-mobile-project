#include <ros/ros.h>
#include <unistd.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include "robmobile_projet/Tab_point.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <math.h>

using namespace ros;
using namespace std;

//Publisher
ros::Publisher pubCmdVel;

//Subscriber
ros::Subscriber sub_odom;

nav_msgs::Odometry odometry;
double xrobot_recu, yrobot_recu;
double theta_robot;

bool arrived,fini,align;

//-------------------------------------------------classe controleROBOT---------------------------------------------//

void calcul_vitesse(geometry_msgs::Point target){
	//double dist_x = (target.x-odometry.pose.pose.position.x);
	//double dist_y = (target.y-odometry.pose.pose.position.y);
	double dist_x =(target.x-xrobot_recu);
	double dist_y =(target.y-yrobot_recu);
	double norme = sqrt(dist_x*dist_x + dist_y*dist_y );
	
	double Kp=0.5;
	double Kp_angle=1;
	float MAX_SPEED_ROT = 3.1414/2; //1/4 tr/sec
	float MAX_SPEED_TRANS = 0.2;  // 20 cm/sec
	
	//Ajout partie gestion d'obstacle


	//partie vitesse angulaire
	/*tf::Quaternion q(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	ROS_INFO("angle robot : %1.2f \n",yaw);
	double theta_robot=yaw;*/
	if(theta_robot != theta_robot){   //odd property of Nan values. On l'utilise pr detecter les Nan (return true qd Nan)
		odometry.twist.twist.angular.z = 0;
		odometry.twist.twist.linear.x = 0;
		return;
	}
	double dif_angle = 0, angle_target=0;
	printf("Erreur par rapport à la target x:%lf y:%lf\n",dist_x,dist_y);
	printf("On est encore à %lf m  de la target\n",norme);

	if(abs(dist_x) > 0.02){
		angle_target = atan2(dist_y,dist_x) ;
		dif_angle = angle_target - theta_robot;
	}else{
		if(dist_y>=0){
			angle_target = 3.1415/2.0;
		}else{
			angle_target = -3.1415/2.0;
		}
		dif_angle = angle_target - theta_robot;
	}
	if(abs(dif_angle)>=3.1415){  //on prend l'angle relatif entre -pi;pi
		if(dif_angle>0){
		dif_angle = dif_angle - 2*3.1415 ;
		}else{
			dif_angle = dif_angle + 2*3.1415 ;
		}
	}

	if(dif_angle < 3.1415/6.0){ 
		odometry.twist.twist.angular.z = Kp_angle*dif_angle ; //correcteur Proportionnel sur l'erreur angulaire
	}else{ //On fait plus d'écart si l'angle est plus grand que pi/6
		odometry.twist.twist.angular.z = MAX_SPEED_ROT * dif_angle/abs(dif_angle);
	}

	printf("angle_target : %lf rotation envoyee : %lf\n",angle_target,odometry.twist.twist.angular.z); //angle next target:0.1253

	//partie vitesse lineaire
	align = (abs(dif_angle) < 3.1415/12.0);  //si on est a moins de pi/12 de la cible on peut commencer a avancer
	if(norme < 0.2 ){  //si on est a 20cm pres de la cible
		arrived = true; //on peut passer a la prochaine cible du chemin
		printf("UN POINT ATTEINT \n \n");
	}else{
		if(abs(dif_angle) > 3.1415/3.0){
			printf("On est en rotation pure!\n");
			odometry.twist.twist.linear.x = 0; // on a beaucoup trop d'écart on doit s'arreter et faire une rotation pure
		}
		else{
			if(align){
				printf("On est alignés, on peut aller tout droit!\n");
				odometry.twist.twist.linear.x = MAX_SPEED_TRANS ; // on est alignés, on peut aller vite à la cible suivante
			}
			else{
				printf("On est en virage, mais on avance quand meme !\n");
				odometry.twist.twist.linear.x = MAX_SPEED_TRANS* abs(3.1415/3.0 - dif_angle)/(3.1415/3.0) ; //plus dif_angle est grand et moins on va vite
			}
		}
	}
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg){
      //rajouter les vitesses et stamped
  odometry.pose.pose.position.x = msg->pose.pose.position.x;
  odometry.pose.pose.position.y = msg->pose.pose.position.y;
  odometry.pose.pose.position.z = msg->pose.pose.position.z;
  odometry.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  odometry.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  odometry.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  odometry.pose.pose.orientation.w = msg->pose.pose.orientation.w;
      //printf("j'ai entendu x:%lf y:%lf \n",odometry.pose.pose.position.x,odometry.pose.pose.position.y);
}

int main( int argc, char** argv ){
    ros::init(argc, argv, "suivi_chemin_test");
	NodeHandle noeud_suivi;
	
	

    //--------------------------On fait un publisher sur /cmd_vel pour le controle robot-----------------------------//
	Publisher pub_cmd_vel = noeud_suivi.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	int indice_avance=0;
    //--------------------------On fait un subscriber sur /odom pour la position courrante du robot-----------------------------//
	arrived = false; fini = false;
	sub_odom=noeud_suivi.subscribe<nav_msgs::Odometry>("/odom",100, odomCallback);
	//en fait non, on écoute la tf
	
	
	//-----------------------------On recoit le chemin ------------------------------------------------------------------------//
	robmobile_projet::Tab_point::ConstPtr chemin_message = ros::topic::waitForMessage<robmobile_projet::Tab_point>("/tab_chemin");
	ROS_INFO("On a recu un tableau, premier point: X(0)= %f, Y(0) = %f, size tab= %d\n",chemin_message->tab_points_X[0],chemin_message->tab_points_Y[0],chemin_message->size);
	ROS_INFO("On a recu un tableau, deuxieme: X(1)= %f, Y(1) = %f, size tab= %d\n",chemin_message->tab_points_X[1],chemin_message->tab_points_Y[1],chemin_message->size);
	ROS_INFO("On a recu un tableau, dernier point : X(fin)= %f, Y(fin) = %f, size tab= %d\n",chemin_message->tab_points_X[chemin_message->size-1],chemin_message->tab_points_Y[chemin_message->size-1],chemin_message->size);
	
	geometry_msgs::Point objectif_int;
	
    //--------------------------On publie sur les topics-----------------------------------------------------------//
	ros::Rate rate(10);
    //int cpt=0;
	while (ros::ok() && fini != true){
		printf("On en est au point du chemin %d\n", indice_avance);
		objectif_int.x=chemin_message->tab_points_X[indice_avance];
		objectif_int.y=chemin_message->tab_points_Y[indice_avance];
		printf("L'objectif est : x = %f, y = %f\n", objectif_int.x,objectif_int.y);
		//printf("On est a la position : x = %f, y = %f\n", odometry.pose.pose.position.x,odometry.pose.pose.position.y);
		//On récupère la position actuelle du robot 
		tf::TransformListener listener;
		int pos_recue=0;
		while(ros::ok() && pos_recue==0){ 
			tf::StampedTransform transform;
			try{
				//listener.lookupTransform("/static_map","/base_link",ros::Time(0), transform);
				listener.lookupTransform("/map","/base_link",ros::Time(0), transform);
				xrobot_recu=transform.getOrigin().x();
				yrobot_recu=transform.getOrigin().y();
				tf::Quaternion q = transform.getRotation();
				theta_robot = tf::getYaw(q);
				pos_recue=1;
				break;
			}
			catch (tf::TransformException &ex) {
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}	
		}
		printf("On est a la position : x = %f, y = %f, angle : %f\n",xrobot_recu,yrobot_recu,theta_robot);
		
		calcul_vitesse(objectif_int);
		pub_cmd_vel.publish(odometry.twist.twist);
		ros::spinOnce();
		rate.sleep();

		if(arrived){  //si on a atteint un objectif du path
			indice_avance++;  //on passe a l'indice suivant
			arrived = false;  //et on dit qu'on repart
		}  
		if(indice_avance >= chemin_message->size){   // si on est arrivé au dernier objectif c'est fini
			odometry.twist.twist.linear.x = 0;
			odometry.twist.twist.angular.z=0;
			fini = true;
		}
      //cpt++;
	}

	pub_cmd_vel.publish(odometry.twist.twist);
	printf("Algorithme Terminé\n");
	return 0;
}
