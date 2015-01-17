#include <iostream>
#include <sstream>
#include <fstream> 
#include <unistd.h>
#include <termios.h>

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <Eigen/Geometry> 


using std::cout;
using std::endl;

char getch() {
     char buf = 0;
     struct termios old = {0};
     if (tcgetattr(0, &old) < 0)
          perror("tcsetattr()");
     old.c_lflag &= ~ICANON;
     old.c_lflag &= ~ECHO;
     old.c_cc[VMIN] = 1;
     old.c_cc[VTIME] = 0;
     if (tcsetattr(0, TCSANOW, &old) < 0)
          perror("tcsetattr ICANON");
     if (read(0, &buf, 1) < 0)
          perror ("read()");
     old.c_lflag |= ICANON;
     old.c_lflag |= ECHO;
     if (tcsetattr(0, TCSADRAIN, &old) < 0)
          perror ("tcsetattr ~ICANON");
     return (buf);
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "velocity_control_node");
     ros::NodeHandle n_;

     // Notify roscore of joystick publication
     ros::Publisher twist_pub_ = n_.advertise<geometry_msgs::Twist>("vel_cmd", 1);     

     ros::ServiceClient client = n_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"); 
     gazebo_msgs::GetModelState getmodelstate;
     getmodelstate.request.model_name="eye";

     //ros::Rate loop_rate(10);
     bool quit = false;
     
     double linear_mag = 1.0;
     double angular_mag = 50.0;
     if (argc >= 3) {          
          linear_mag = atof(argv[1]);
          angular_mag = atof(argv[2]);
          //std::stringstream lin_mag_convert(std::string(argv[1]));
          //std::stringstream ang_mag_convert(std::string(argv[2]));
          //lin_mag_convert >> linear_mag;
          //angular_mag_convert >> angular_mag;
     }     

     Eigen::Quaternion<float> forward(0,0,0,0);

     cout << "Issue TWIST messages with W A S D and the arrow keys." << endl;
     while (ros::ok() && !quit) {          
          geometry_msgs::Twist msg;
          msg.linear.x = 0.0;
          msg.linear.y = 0.0;
          msg.linear.z = 0.0;
          
          msg.angular.x = 0.0;
          msg.angular.y = 0.0;
          msg.angular.z = 0.0;
          
          twist_pub_.publish(msg);
          
          Eigen::Quaternion<float> attitude;
          if(client.call(getmodelstate)) {
               //std::cout << getmodelstate.response.pose << std::endl;  
               attitude = Eigen::Quaternion<float>(getmodelstate.response.pose.orientation.w,
                                                   getmodelstate.response.pose.orientation.x,
                                                   getmodelstate.response.pose.orientation.y,
                                                   getmodelstate.response.pose.orientation.z);
          } else {
               cout << "Error getting model state." << endl;
               attitude = forward;
          }                                        
          
          //Eigen::Quaternion<float> direction;

          Eigen::Quaternion<float>::Vector3 direction(0,0,0);
          Eigen::Quaternion<float>::Vector3 rotation(0,0,0);          

          char c, c2;          
          c = getch();         
          switch(c) {
          case 'w':
               direction = attitude._transformVector(Eigen::Quaternion<float>::Vector3(linear_mag,0,0));              
                    
               //q = Eigen::AngleAxis<float>(,);
//msg.linear.x = linear_mag;
               break;
          case 'a':
               direction = attitude._transformVector(Eigen::Quaternion<float>::Vector3(0,linear_mag,0));               
               
               //msg.linear.y = linear_mag;
               break;
          case 's':
               direction = attitude._transformVector(Eigen::Quaternion<float>::Vector3(-linear_mag,0,0));               
               
               //msg.linear.x = -linear_mag;
               break;
          case 'd':
               direction = attitude._transformVector(Eigen::Quaternion<float>::Vector3(0,-linear_mag,0));               
               
               //msg.linear.y = -linear_mag;
               break;
          case 'q':
               rotation = attitude._transformVector(Eigen::Quaternion<float>::Vector3(angular_mag,0,0));
               break;
          case 'e':
               rotation = attitude._transformVector(Eigen::Quaternion<float>::Vector3(-angular_mag,0,0));
               break;
          case 27:
               if (getch() == '[') {
                    c2 = getch();
                    switch(c2) {
                    case 'D':
                         // LEFT ARROW
                         //msg.angular.z = angular_mag;
                         rotation = attitude._transformVector(Eigen::Quaternion<float>::Vector3(0,0,angular_mag));
                         break;
                    case 'C':
                         // RIGHT ARROW
                         //msg.angular.z = -angular_mag;
                         rotation = attitude._transformVector(Eigen::Quaternion<float>::Vector3(0,0,-angular_mag));
                         break;
                    case 'A':
                         //msg.angular.y = angular_mag;
                         rotation = attitude._transformVector(Eigen::Quaternion<float>::Vector3(0,angular_mag,0));
                         // UP ARROW
                         break;
                    case 'B':
                         //msg.angular.y = -angular_mag;
                         rotation = attitude._transformVector(Eigen::Quaternion<float>::Vector3(0,-angular_mag,0));
                         // DOWN ARROW
                         break;
                    default:
                         break;
                    }
               }
               break;
          default:
               break;
          }

          msg.linear.x = direction.x();
          msg.linear.y = direction.y();
          msg.linear.z = direction.z();

          msg.angular.x = rotation.x();
          msg.angular.y = rotation.y();
          msg.angular.z = rotation.z();
          //printf("Got: %c=%d\n",c,c);
          
          twist_pub_.publish(msg);
          usleep(1000);
          
          //ros::spinOnce();
          //loop_rate.sleep();
     }
     
     return 0;
}
