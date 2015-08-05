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

//void cloudCallback(const sensor_msgs::PointCloudConstPtr& msg)
//{
//     cloud_in_ = *msg;
//
//     tf::TransformListener listener;
//     tf::StampedTransform transform;
//     try{
//          listener.transformPointCloud("sonar_link", cloud_in_, cloud_);
//     }
//     catch (tf::TransformException ex){
//          ROS_ERROR("%s",ex.what());     
//          return;
//     }
//
//     if (cloud_.points.size () == 0) {
//          cout << "Point Cloud of size 0" << endl;
//          return;
//     }
//
//     /////////////////////////////////////////////
//     // Add Gaussian Distribution to PointCloud (in range direction)
//     /////////////////////////////////////////////
//     for (unsigned int i = 0; i < cloud_.points.size(); i++) {
//          cloud_.points[i].x += gauss_sample();
//          //cloud_.points[i].y += gauss_sample();
//          //cloud_.points[i].z += gauss_sample();
//     }     
//
//     // DEBUG OUTPUT
//     //cout << "Channels: " << cloud_.channels.size() << endl;
//     //for (unsigned int i = 0; i < cloud_.channels.size(); i++) {
//     //     cout << endl << "---------------" << endl;
//     //     cout << "Channel: " << i << endl;
//     //     cout << "Name: " << cloud_.channels[i].name << endl;
//     //     cout << "Values: ";
//     //     for(unsigned int j = 0; j < cloud_.channels[i].values.size(); j++) {
//     //          cout << cloud_.channels[i].values[j] << ", ";
//     //     }
//     //}
//
//     /////////////////////////////////////////////
//     // Convert range data to opencv image
//     /////////////////////////////////////////////
//     double z_min = 99999, z_max = -99999, x_min = 99999, x_max = -99999, y_min = 99999, y_max = -99999;
//     for (unsigned int i = 0; i < cloud_.points.size(); i++) {
//          if (cloud_.points[i].z < z_min) {
//               z_min = cloud_.points[i].z;
//          }
//          
//          if (cloud_.points[i].z > z_max) {
//               z_max = cloud_.points[i].z;
//          }
//
//          if (cloud_.points[i].x < x_min) {
//               x_min = cloud_.points[i].x;
//          }
//
//          if (cloud_.points[i].x > x_max) {
//               x_max = cloud_.points[i].x;
//          }
//
//          if (cloud_.points[i].y < y_min) {
//               y_min = cloud_.points[i].y;
//          }
//
//          if (cloud_.points[i].y > y_max) {
//               y_max = cloud_.points[i].y;
//          }
//     }
//
//     //cout << "=================" << endl;
//     //cout << "z_min: " << z_min << endl;
//     //cout << "z_max: " << z_max << endl;
//     //cout << "x_min: " << x_min << endl;
//     //cout << "x_max: " << x_max << endl;
//     //cout << "y_min: " << y_min << endl;
//     //cout << "y_max: " << y_max << endl;
//     
//     // x-direction: surge direction
//     // y-direction: yaw direction
//     // z-direction: heave direction
//     
//     double scale = 32;
//     double x_diff = x_max - x_min; // sonar range
//     double y_diff = y_max - y_min; // related to angle
//     double z_diff = z_max - z_min; // disappears
//     
//     double x_off = -x_min;
//     double y_off = -y_min;
//     
//     cv::Mat img = cv::Mat::zeros(y_diff*scale,x_diff*scale,CV_8UC3);          
//     for (unsigned int i = 0; i < cloud_.points.size(); i++) {          
//          int x_pos = img.rows - (cloud_.points[i].y + y_off)*scale;
//          int y_pos = (cloud_.points[i].x + x_off)*scale;
//          
//          if (x_pos < 0 || x_pos > img.rows || y_pos < 0 || y_pos > img.cols) {
//               // error, skip
//               cout << "bounds" << endl;
//               continue;
//          }          
//          double retro = cloud_.channels[0].values[i];
//          if (retro > 255) { 
//               retro = 255;
//          } else if (retro < 0) {
//               retro = 0;
//          }
//          
//          int R = retro;
//          int G = retro;
//          int B = retro;
//          //cout << retro << ", ";
//          cv::circle(img,cv::Point(x_pos, y_pos),1,cv::Scalar(B,G,R),1,8,0);
//     }
//
//     cv::Point center = cv::Point( img.cols/2, img.rows/2 );
//     double angle = 180.0;
//     double rot_scale = 1.0;
//     
//     cv::Mat rot_mat(2,3,CV_8UC3);
//     rot_mat = cv::getRotationMatrix2D( center, angle, rot_scale );
//     cv::warpAffine(img, img, rot_mat, img.size());
//     
//     //cv::imshow("sonar_image",img);
//     //cv::waitKey(1);
//     
//     sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), 
//                                                        "bgr8",
//                                                        img).toImageMsg();
//     
//     img_pub_.publish(img_msg);
//     
//}
int scale(double c)
{
     if (c < 0) {
          return 0;
     } else if (c > 1) {
          return 255;
     }
     return (c / 1.0) * 255;     
}

/*
   Return a RGB colour value given a scalar v in the range [vmin,vmax]
   In this case each colour component ranges from 0 (no contribution) to
   1 (fully saturated), modifications for other ranges is trivial.
   The colour is clipped at the end of the scales if v is outside
   the range [vmin,vmax]
*/

typedef struct color{
    double r,g,b;
} color_t;

color_t GetColor(double v,double vmin,double vmax)
{
   color_t c = {1.0,1.0,1.0}; // white
   double dv;

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c.r = 0;
      c.g = 4 * (v - vmin) / dv;
   } else if (v < (vmin + 0.5 * dv)) {
      c.r = 0;
      c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
   } else if (v < (vmin + 0.75 * dv)) {
      c.r = 4 * (v - vmin - 0.5 * dv) / dv;
      c.b = 0;
   } else {
      c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      c.b = 0;
   }

   return(c);
}

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
     }     

     double x_min = 0.1;
     double x_max = 30.0;
     double y_min = 0;
     double y_max = sin(0.392699082)*x_max*2;
     
     int img_width = 600;
     int img_height = 600;

     cv::Mat img = cv::Mat::zeros(img_height, img_width, CV_8UC3);          

     // Draw edges of sonar image
     double start_angle = 1.178097245;
     double end_angle = 1.963495408;
     cv::Point start(img_width/2,0);
     cv::Point end_1(start.x+img_height*cos(start_angle), start.y+img_height*sin(start_angle));
     cv::Point end_2(start.x+img_height*cos(end_angle), start.y+img_height*sin(end_angle));
     
     std::vector<cv::Point> contour;
     contour.push_back(start);
     contour.push_back(end_1);
     
     double num_pts = 50;
     double step = (end_angle - start_angle) / num_pts;
     for (double ang = start_angle; ang <= end_angle; ang += step) {
          cv::Point p();
          contour.push_back(cv::Point(start.x+img_height*cos(ang), start.y+img_height*sin(ang)));
     }
     
     contour.push_back(end_2);     
     
     // create a pointer to the data as an array of points (via a conversion to 
     // a Mat() object)
     const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
     int npts = cv::Mat(contour).rows;

     // Fill the polygon with "empty" "blue" returns     
     color_t color = GetColor(0,0,255);     
     cv::fillPoly(img, &pts, &npts, 1, cv::Scalar(scale(color.b), scale(color.g), scale(color.r)), 8, 0, cv::Point(0,0));
                  
     // Add the returned values
     for (unsigned int i = 0; i < cloud_.points.size(); i++) {          

          //cout << cloud_.points[i].x << ", " << cloud_.points[i].y << endl;

          if (cloud_.points[i].x > x_max) {
               cout << "Over x_max: " << cloud_.points[i].x << endl;
          }

          if (cloud_.points[i].y > y_max) {
               cout << "Over y_max: " << cloud_.points[i].y << endl;
          }
          
          int x_pos = img_width/2 + -cloud_.points[i].y / (y_max/2) * img_height*sin(0.392699082);
          int y_pos = cloud_.points[i].x / x_max * img_height;
                    
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
          
          color_t color = GetColor(retro,0,255);
          int R = scale(color.r);
          int G = scale(color.g);
          int B = scale(color.b);
          //cout << retro << ", ";
          cv::circle(img,cv::Point(x_pos, y_pos),1,cv::Scalar(B,G,R),1,8,0);
     }
     
     // Rotate the sonar image to pointing "up"
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
