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

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
//#include <pcl/PCLPointCloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/point_cloud_conversion.h>

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

//typedef struct{
//    double r,g,b;
//} color_t;
struct color_t {
     double r;
     double g;
     double b;
};

struct color_t GetColor(double v, double vmin, double vmax)
{
   struct color_t c = {1.0,1.0,1.0}; // white
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
     double x_max = 10.0;
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
     struct color_t color = GetColor(0,0,255);     
     cv::fillPoly(img, &pts, &npts, 1, cv::Scalar(scale(color.b), scale(color.g), scale(color.r)), 8, 0, cv::Point(0,0));
   
#if 1
     sensor_msgs::PointCloud2 sm_pcl2;
     sensor_msgs::convertPointCloudToPointCloud2(cloud_, sm_pcl2);
     
     pcl::PCLPointCloud2 pcl_pc2;
     pcl_conversions::toPCL(sm_pcl2, pcl_pc2);
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
     
     // Normal estimation*
     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
     tree->setInputCloud (cloud);
     n.setInputCloud (cloud);
     n.setSearchMethod (tree);
     n.setKSearch (20);
     n.compute (*normals);

     // Concatenate the XYZ and normal fields*
     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
     pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
     //* cloud_with_normals = cloud + normals
     
     // Create search tree*
     pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
     tree2->setInputCloud (cloud_with_normals);

     // Initialize objects
     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
     pcl::PolygonMesh triangles;

     // Set the maximum distance between connected points (maximum edge length)
     //gp3.setSearchRadius (0.025);
     gp3.setSearchRadius (15);

     // Set typical values for the parameters
     gp3.setMu (2.5);
     gp3.setMaximumNearestNeighbors (100);
     gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
     gp3.setMinimumAngle(M_PI/18); // 10 degrees
     gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
     gp3.setNormalConsistency(false);

     // Get result
     gp3.setInputCloud (cloud_with_normals);
     gp3.setSearchMethod (tree2);
     gp3.reconstruct (triangles);

     // Additional vertex information
     //std::vector<int> parts = gp3.getPartIDs();
     //std::vector<int> states = gp3.getPointStates();                    

     struct color_t mycolor;
     int B,G,R;

     std::vector<pcl::Vertices>::iterator it = triangles.polygons.begin();
     for(; it != triangles.polygons.end(); it++) {
          pcl::Vertices verts = *it;                    
          
          std::vector<uint32_t>::iterator it2 = verts.vertices.begin();          
          int count = 0;
          double retro_sum = 0;
          cv::Point points[0][3];
          for (; it2 != verts.vertices.end(); it2++) {
               int x_pos = img_width/2 + -cloud->points[*it2].y / (y_max/2) * img_height*sin(0.392699082);
               int y_pos = cloud->points[*it2].x / x_max * img_height;               
               
               points[0][count++] = cv::Point( x_pos, y_pos );               
               
               retro_sum += cloud_.channels[0].values[*it2];
               
               mycolor = GetColor(cloud_.channels[0].values[*it2], 0, 255);
               R = scale(mycolor.r);
               G = scale(mycolor.g);
               B = scale(mycolor.b);
               
               cv::circle(img,cv::Point(x_pos, y_pos),1,cv::Scalar(B,G,R),1,8,0);
          }                    
          
          const cv::Point* ppt[1] = { points[0] };
          int npt[] = { 3 };
          
          double retro = retro_sum / 3.0;
          if (retro > 255) { 
               retro = 255;
          } else if (retro < 0) {
               retro = 0;
          }
          
          mycolor = GetColor(retro, 0, 255);
          R = scale(mycolor.r);
          G = scale(mycolor.g);
          B = scale(mycolor.b);
                    
          //cout << B << "," << G << "," << R << endl;                    

          cv::fillPoly( img,
                        ppt,
                        npt,
                        1,
                        cv::Scalar(B,G,R),
                        8,
                        0,
                        cv::Point(0,0));
     }                    
     
#else             
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
#endif

     cv::medianBlur(img,img,11);
     
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

     ros::Subscriber cloud_sub = n_.subscribe<sensor_msgs::PointCloud>("sonar_cloud", 0, cloudCallback);
     img_pub_ = it_.advertise("sonar_image", 1);         
     
     ros::Rate loop_rate(10);     
     while (ros::ok()) {                    
          ros::spinOnce();
          //loop_rate.sleep();
     }

     return 0;
}
