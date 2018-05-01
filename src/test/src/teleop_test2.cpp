#include "ros/ros.h"
#include <termios.h>
#include <geometry_msgs/Twist.h>

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;

}


class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;

public:

  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard()
  {

     std::cout << "Type a command and then press enter.  "
       "Use 'z' to move forward,Use 'x' to move backward 'q' to turn left, "
       "'d' to turn right, 's' to stop the robot, 'k' to exit.\n";
     
     
     //we will be sending commands of type "twist"
     geometry_msgs::Twist base_cmd;
	
ros::Rate loop_rate(10);
    char cmd[50]= "z";
    int c=0;
    
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    while(nh_.ok()){

      // std::cin.getline(cmd, 50);
      cmd[0] = getch();

      if(cmd[0]!='z' && cmd[0]!='q' && cmd[0]!='d' && cmd[0]!='s'&& cmd[0]!='k'&& cmd[0]!='x')
      {
        std::cout << "unknown command:" << cmd << "\n";
        continue;
      }

      //move forward
       if(cmd[0]=='z'){
        base_cmd.linear.x += 0.1;
      } 
      //move backward
      if(cmd[0]=='x'){
        base_cmd.linear.x += -0.1;
      } 
      //turn left (yaw) 
      else if(cmd[0]=='q'){
        base_cmd.angular.z += 0.75;
        base_cmd.linear.x += 0;
      } 
      //turn right (yaw) and drive forward at the same time
      else if(cmd[0]=='d'){
        base_cmd.angular.z += -0.75;
        base_cmd.linear.x += 0;
	}
      //Stoppe les moteurs
      else if(cmd[0]=='s'){ 
        base_cmd.angular.z = 0;
        base_cmd.linear.x = 0;
      } 
      //quit
      else if(cmd[0]=='k'){
        break;
      }*/
      
      //publish the assembled command
      cmd_vel_pub_.publish(base_cmd);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return true;
  }
};


int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  driver.driveKeyboard();
}
