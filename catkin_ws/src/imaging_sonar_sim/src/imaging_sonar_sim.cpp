#include "ros/ros.h"

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>

#include <boost/math/distributions/normal.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

using std::cout;
using std::endl;

sensor_msgs::PointCloud cloud_in_;
sensor_msgs::PointCloud cloud_;

image_transport::Publisher img_pub_;

// Normal Distribution Setup
typedef boost::mt19937                     ENG;    // Mersenne Twister
typedef boost::normal_distribution<double> DIST;   // Normal Distribution
typedef boost::variate_generator<ENG,DIST> GEN;    // Variate generator
ENG  eng;
DIST dist(0,0.01); // mean, std
GEN  gauss_sample(eng,dist);

void cloudCallback(const sensor_msgs::PointCloudConstPtr& msg)
{
     cloud_in_ = *msg;

     tf::TransformListener listener;
     tf::StampedTransform transform;
     try{
          listener.transformPointCloud("sonar_link", cloud_in_, cloud_);
     }
     catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());     
          return;
     }

     if (cloud_.points.size () == 0) {
          cout << "Point Cloud of size 0" << endl;
          return;
     }

     /////////////////////////////////////////////
     // Add Gaussian Distribution to PointCloud (in range direction)
     /////////////////////////////////////////////
     for (unsigned int i = 0; i < cloud_.points.size(); i++) {
          cloud_.points[i].x += gauss_sample();
          //cloud_.points[i].y += gauss_sample();
          //cloud_.points[i].z += gauss_sample();
     }     

     // DEBUG OUTPUT
     //cout << "Channels: " << cloud_.channels.size() << endl;
     //for (unsigned int i = 0; i < cloud_.channels.size(); i++) {
     //     cout << endl << "---------------" << endl;
     //     cout << "Channel: " << i << endl;
     //     cout << "Name: " << cloud_.channels[i].name << endl;
     //     cout << "Values: ";
     //     for(unsigned int j = 0; j < cloud_.channels[i].values.size(); j++) {
     //          cout << cloud_.channels[i].values[j] << ", ";
     //     }
     //}

     /////////////////////////////////////////////
     // Convert range data to opencv image
     /////////////////////////////////////////////
     double z_min = 99999, z_max = -99999, x_min = 99999, x_max = -99999, y_min = 99999, y_max = -99999;
     for (unsigned int i = 0; i < cloud_.points.size(); i++) {
          if (cloud_.points[i].z < z_min) {
               z_min = cloud_.points[i].z;
          }
          
          if (cloud_.points[i].z > z_max) {
               z_max = cloud_.points[i].z;
          }

          if (cloud_.points[i].x < x_min) {
               x_min = cloud_.points[i].x;
          }

          if (cloud_.points[i].x > x_max) {
               x_max = cloud_.points[i].x;
          }

          if (cloud_.points[i].y < y_min) {
               y_min = cloud_.points[i].y;
          }

          if (cloud_.points[i].y > y_max) {
               y_max = cloud_.points[i].y;
          }
     }

     //cout << "=================" << endl;
     //cout << "z_min: " << z_min << endl;
     //cout << "z_max: " << z_max << endl;
     //cout << "x_min: " << x_min << endl;
     //cout << "x_max: " << x_max << endl;
     //cout << "y_min: " << y_min << endl;
     //cout << "y_max: " << y_max << endl;
     
     // x-direction: surge direction
     // y-direction: yaw direction
     // z-direction: heave direction
     
     double scale = 32;
     double x_diff = x_max - x_min; // sonar range
     double y_diff = y_max - y_min; // related to angle
     double z_diff = z_max - z_min; // disappears
     
     double x_off = -x_min;
     double y_off = -y_min;
     
     cv::Mat img = cv::Mat::zeros(y_diff*scale,x_diff*scale,CV_8UC3);          
     for (unsigned int i = 0; i < cloud_.points.size(); i++) {          
          int x_pos = img.rows - (cloud_.points[i].y + y_off)*scale;
          int y_pos = (cloud_.points[i].x + x_off)*scale;
          
          if (x_pos < 0 || x_pos > img.rows || y_pos < 0 || y_pos > img.cols) {
               // error, skip
               cout << "bounds" << endl;
               continue;
          }          
          double retro = cloud_.channels[0].values[i];
          if (retro > 255) { 
               retro = 255;
          } else if (retro < 0) {
               retro = 0;
          }
          
          int R = retro;
          int G = retro;
          int B = retro;
          //cout << retro << ", ";
          cv::circle(img,cv::Point(x_pos, y_pos),1,cv::Scalar(B,G,R),1,8,0);
     }

     cv::Point center = cv::Point( img.cols/2, img.rows/2 );
     double angle = 180.0;
     double rot_scale = 1.0;
     
     cv::Mat rot_mat(2,3,CV_8UC3);
     rot_mat = cv::getRotationMatrix2D( center, angle, rot_scale );
     cv::warpAffine(img, img, rot_mat, img.size());
     
     //cv::imshow("sonar_image",img);
     //cv::waitKey(1);
     
     sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), 
                                                        "bgr8",
                                                        img).toImageMsg();
     
     img_pub_.publish(img_msg);
     
}

int main(int argc, char * argv[])
{     
     ros::init(argc, argv, "imaging_sonar_sim");
     ros::NodeHandle n_;
     image_transport::ImageTransport it_(n_);

     ros::Subscriber cloud_sub = n_.subscribe<sensor_msgs::PointCloud>("sonar_cloud", 1, cloudCallback);     
     img_pub_ = it_.advertise("sonar_image", 1);         
     
     ros::Rate loop_rate(10);     
     while (ros::ok()) {                    

          ros::spinOnce();
          loop_rate.sleep();
     }

     return 0;
}
