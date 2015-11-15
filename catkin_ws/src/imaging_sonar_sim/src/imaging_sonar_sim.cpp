#include "ros/ros.h"

#include <iostream>

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>

#include <opencv2/core/core.hpp>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>

#include <boost/math/distributions/normal.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace fs = boost::filesystem;

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

#include <tinyxml.h>

#include "ColorMaps.h"

using std::cout;
using std::endl;

sensor_msgs::PointCloud cloud_in_;
sensor_msgs::PointCloud cloud_;

image_transport::Publisher img_pub_;

ros::Publisher camera_info_pub_;
sensor_msgs::CameraInfo camera_info_;

//cv::VideoWriter record_;
int frame_count = 0;
std::string output_dir = "/home/syllogismrxs/temp/sonar";

#define PI 3.14159265359

// Normal Distribution Setup
typedef boost::mt19937                     ENG;    // Mersenne Twister
typedef boost::normal_distribution<double> DIST;   // Normal Distribution
typedef boost::variate_generator<ENG,DIST> GEN;    // Variate generator
ENG  eng;
DIST dist(0,0.01); // mean, std
GEN  gauss_sample(eng,dist);

void add_salt_and_pepper(cv::Mat& I, int amt)
{
     // accept only char type matrices
     CV_Assert(I.depth() != sizeof(uchar));

     //struct color_t c;
     
     const int channels = I.channels();
     switch(channels) {
     case 1: {
          cv::MatIterator_<uchar> it, end;
          for( it = I.begin<uchar>(), end = I.end<uchar>(); it != end; ++it) {
               *it += rand() % amt;
          }          
          break;
     }
     case 3: {
          cv::MatIterator_<cv::Vec3b> it, end;
          for( it = I.begin<cv::Vec3b>(), end = I.end<cv::Vec3b>(); it != end; ++it) {
               //(*it)[0] += scale(c.b); // B
               //(*it)[1] += scale(c.g); // G
               //(*it)[2] += scale(c.r); // R
          }
     }}    
}


double min_angle_;
double max_angle_;
double beam_width_;

std::string sonar_link_name;

void cloudCallback(const sensor_msgs::PointCloudConstPtr& msg)
{
     cloud_in_ = *msg;
     
     tf::TransformListener listener;
     tf::StampedTransform transform;
     try{
          listener.transformPointCloud(sonar_link_name, cloud_in_, cloud_);
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
     
     double x_min = 0.1;  // R_min
     double x_max = 10.0; // R_max
     double y_min = 0;    // 
     //double y_max = sin(0.392699082)*x_max*2;
     double y_max = sin(max_angle_)*x_max*2;
     
     int img_width = 600;
     //int img_width = 1500;
     int img_height = 600;

     camera_info_.height = img_height;
     camera_info_.width = img_width;
     
     cv::Mat img = cv::Mat::zeros(img_height, img_width, CV_8UC1);          
     
     //add_salt_and_pepper(img, 10);
     
#if 0
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

     std::vector<pcl::Vertices>::iterator it = triangles.polygons.begin();
     for(; it != triangles.polygons.end(); it++) {
          pcl::Vertices verts = *it;                    
          
          std::vector<uint32_t>::iterator it2 = verts.vertices.begin();          
          int count = 0;
          double retro_sum = 0;
          cv::Point points[0][3];
          for (; it2 != verts.vertices.end(); it2++) {
               //int x_pos = img_width/2 + -cloud->points[*it2].y / (y_max/2) * img_height*sin(0.392699082);
               int x_pos = img_width/2 + -cloud->points[*it2].y / (y_max/2) * img_height*sin(max_angle_);
               int y_pos = cloud->points[*it2].x / x_max * img_height;               
               
               points[0][count++] = cv::Point( x_pos, y_pos );               
               
               retro_sum += cloud_.channels[0].values[*it2];
               
               cv::circle(img,cv::Point(x_pos, y_pos),1, cloud_.channels[0].values[*it2], 1,8,0);
          }                    
          
          const cv::Point* ppt[1] = { points[0] };
          int npt[] = { 3 };
          
          double retro = retro_sum / 3.0;
          if (retro > 255) { 
               retro = 255;
          } else if (retro < 0) {
               retro = 0;
          }
          
          cv::fillPoly( img,
                        ppt,
                        npt,
                        1,
                        retro,
                        8,
                        0,
                        cv::Point(0,0));
     }                    
     
#else             
     // Add the returned values
     for (unsigned int i = 0; i < cloud_.points.size(); i++) {
          if (cloud_.points[i].x > x_max) {
               cout << "Over x_max: " << cloud_.points[i].x << endl;
          }

          if (cloud_.points[i].y > y_max) {
               cout << "Over y_max: " << cloud_.points[i].y << endl;
          }
          
          //int x_pos = img_width/2 + -cloud_.points[i].y / (y_max/2) * img_height*sin(0.392699082);
          int x_pos = img_width/2 + -cloud_.points[i].y / (y_max/2) * img_height*sin(max_angle_);
          int y_pos = cloud_.points[i].x / x_max * img_height;
                    
          if (x_pos < 0 || x_pos > img.cols || y_pos < 0 || y_pos > img.rows) {
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
          cv::circle(img,cv::Point(x_pos, y_pos),1,retro,1,8,0);
     }
#endif

     cv::medianBlur(img,img,3);
     //add_salt_and_pepper(img,20);
     
     cv::Mat img_color;
     //cv::applyColorMap(img,img_color,cv::COLORMAP_JET);          
     /// COLORMAP_AUTUMN
     /// COLORMAP_BONE
     /// COLORMAP_COOL
     /// COLORMAP_HOT
     /// COLORMAP_HSV
     /// COLORMAP_JET
     /// COLORMAP_OCEAN
     /// COLORMAP_PINK
     /// COLORMAP_RAINBOW
     /// COLORMAP_SPRING
     /// COLORMAP_SUMMER
     /// COLORMAP_WINTER
     //cv::applyColorMap(img,img_color,cv::COLORMAP_JET);    
     Gray2Jet_matlab(img, img_color);
     ///////////////////////////////

     //double start_angle = 1.178097245;
     
     double start_angle = (PI - beam_width_) / 2 ;
     double end_angle = 90.0 * PI / 180.0;//1.963495408;
     cv::Point start(img_width/2,0);
     cv::Point end_1(start.x+img_height*cos(start_angle), start.y+img_height*sin(start_angle));
     cv::Point end_2(start.x+img_height*cos(end_angle), start.y+img_height*sin(end_angle));
     
     std::vector<cv::Point> contour;
     contour.push_back(start);
     contour.push_back(end_1);
     
     double num_pts = 25;
     double step = (end_angle - start_angle) / num_pts;
     for (double ang = start_angle; ang <= end_angle; ang += step) {
          contour.push_back(cv::Point(start.x+img_height*cos(ang), start.y+img_height*sin(ang)));
     }
     
     contour.push_back(end_2);     
     contour.push_back(cv::Point(img_width-1,img_height-1));
     contour.push_back(cv::Point(img_width-1,0));     
     
     // create a pointer to the data as an array of points (via a conversion to 
     // a Mat() object)
     const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
     int npts = cv::Mat(contour).rows;

     // Fill the outside of the sonar image with black
     cv::fillPoly(img_color, &pts, &npts, 1, cv::Scalar(0,0,0), 8, 0, cv::Point(0,0));

     ///////////////////

     //start_angle = 112.5  * PI / 180.0;
     start_angle = ((PI - beam_width_) / 2) + beam_width_;
     end_angle = 90 * PI / 180.0;//1.963495408;
     end_1 = cv::Point(start.x+img_height*cos(start_angle), start.y+img_height*sin(start_angle));
     end_2 = cv::Point(start.x+img_height*cos(end_angle), start.y+img_height*sin(end_angle));
     
     contour.clear();
     contour.push_back(start);
     contour.push_back(end_1);
     
     num_pts = 25;
     step = (start_angle - end_angle) / num_pts;
     for (double ang = start_angle; ang >= end_angle; ang -= step) {
          contour.push_back(cv::Point(start.x+img_height*cos(ang), start.y+img_height*sin(ang)));
     }
     
     contour.push_back(end_2);     
     contour.push_back(cv::Point(0,img_height-1));
     contour.push_back(cv::Point(0,0));     
     
     // create a pointer to the data as an array of points (via a conversion to 
     // a Mat() object)
     pts = (const cv::Point*) cv::Mat(contour).data;
     npts = cv::Mat(contour).rows;

     // Fill the outside of the sonar image with black
     cv::fillPoly(img_color, &pts, &npts, 1, 0, 8, 0, cv::Point(0,0));


     ////////////////////

     // Rotate the sonar image to pointing "up"
     cv::Point center = cv::Point( img_color.cols/2, img_color.rows/2 );
     double angle = 180.0;
     double rot_scale = 1.0;
     
     cv::Mat rot_mat(2,3,CV_8UC1);
     rot_mat = cv::getRotationMatrix2D( center, angle, rot_scale );
     cv::warpAffine(img_color, img_color, rot_mat, img_color.size());
     
     //cv::imshow("sonar_image",img);
     //cv::waitKey(1);          
     std_msgs::Header img_header;
     img_header.stamp = ros::Time::now();     
     
     sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(img_header, 
                                                        "bgr8",
                                                        img_color).toImageMsg();
     
     camera_info_.header.stamp = img_header.stamp;
          
     camera_info_pub_.publish(camera_info_);
     img_pub_.publish(img_msg);    

     std::ostringstream convert;
     convert << frame_count;
     
     std::string out_str = output_dir + "/sonar" + convert.str() + ".png";
     cv::imwrite(out_str, img_color);
     frame_count++;
     //if (!record_.isOpened()) {
     //     record_.open("/home/syllogismrxs/output.avi", CV_FOURCC('M','J','P','G'), 10, img_color.size(), true);
     //}
     //
     //if (record_.isOpened()) {
     //     record_ << img_color;
     //}     
}
          
int main(int argc, char * argv[])
{               
     srand (time(NULL));     

     ros::init(argc, argv, "imaging_sonar_sim");
     ros::NodeHandle n_;

     ros::param::get("~sonar_link_name", sonar_link_name);

     std::string key;
     std::string robot_description;
     if (n_.searchParam("robot_description", key)) {
          n_.getParam(key, robot_description);
     } else {
          cout << "===========================" << endl;
          cout << "Warning: unable to find robot_description." << endl;
     }     

     // Create output directory for sonar images if it doesn't exist
     fs::path dir(output_dir);
     if (!fs::is_directory(dir)) {
          fs::create_directories(output_dir);
     }     
     
     // Need to extract the min and max angles for sonar point cloud
     TiXmlDocument doc;
     const char* pTest = doc.Parse(robot_description.c_str(), 0 , TIXML_ENCODING_UTF8);
     if(pTest != NULL){
          cout << "Error parsing robot_description" << endl;
     }
     TiXmlElement * robot_element = doc.FirstChildElement("robot");
     // Search for gazebo with reference="sonar_link"
     TiXmlElement *elem = robot_element->FirstChildElement();
     TiXmlElement *sonar_link_elem;
     while (elem) {
          if (std::string(elem->Value()) == "gazebo") {
               if (elem->Attribute("reference") != NULL) {
                    if (std::string(elem->Attribute("reference")) == sonar_link_name) {
                         sonar_link_elem = elem;
                         break;
                    }
               }
          }
          elem = elem->NextSiblingElement();
     }
     TiXmlElement *min_angle_elem = sonar_link_elem->FirstChildElement("sensor")->FirstChildElement("ray")->FirstChildElement("scan")->FirstChildElement("horizontal")->FirstChildElement("min_angle");
     TiXmlElement *max_angle_elem = sonar_link_elem->FirstChildElement("sensor")->FirstChildElement("ray")->FirstChildElement("scan")->FirstChildElement("horizontal")->FirstChildElement("max_angle");     
     if ( ! (std::istringstream(min_angle_elem->GetText()) >> min_angle_) ) min_angle_ = -1;
     if ( ! (std::istringstream(max_angle_elem->GetText()) >> max_angle_) ) max_angle_ = 1;
     cout << "Min Angle: " << min_angle_ << endl;
     cout << "Max Angle: " << max_angle_ << endl;
     beam_width_ = max_angle_ * 2;
          
     std::string cloud_topic_name;
     ros::param::get("~cloud_topic_name", cloud_topic_name);
          
     // Setup sonar cloud subscription     
     ros::Subscriber cloud_sub = n_.subscribe<sensor_msgs::PointCloud>(cloud_topic_name, 0, cloudCallback);

     // Setup sonar_image publication
     std::string image_topic_name;
     ros::param::get("~image_topic_name", image_topic_name);
          
     image_transport::ImageTransport it_(n_);
     img_pub_ = it_.advertise(image_topic_name, 1);      

     cout << "=======" << endl;
     cout << "cloud name: " << cloud_topic_name << endl;
     cout << "image name: " << image_topic_name << endl;
     cout << "sonar link name: " << sonar_link_name << endl;
     
     camera_info_pub_ = n_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);     
     
     ros::Rate loop_rate(10);     
     while (ros::ok()) {                    
          ros::spinOnce();
          loop_rate.sleep();
     }

     return 0;
}
