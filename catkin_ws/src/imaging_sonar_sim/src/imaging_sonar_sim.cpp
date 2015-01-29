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


using std::cout;
using std::endl;

//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
//#include <boost/foreach.hpp>
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//void callback(const PointCloud::ConstPtr& msg)
//{
//     printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//     BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//          printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
//}

sensor_msgs::PointCloud cloud_in_;
sensor_msgs::PointCloud cloud_out_;
void cloudCallback(const sensor_msgs::PointCloudConstPtr& msg)
{
     cloud_in_ = *msg;

     tf::TransformListener listener;
     tf::StampedTransform transform;
     try{
          listener.transformPointCloud("sonar_link", cloud_in_, cloud_out_);
     }
     catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());     
     }
}

int main(int argc, char * argv[])
{
     ros::init(argc, argv, "imaging_sonar_sim");
     ros::NodeHandle n_;

     ros::Subscriber cloud_sub = n_.subscribe<sensor_msgs::PointCloud>("sonar_cloud", 1, cloudCallback);     

     ros::Rate loop_rate(10);     
     while (ros::ok()) {                    

          ros::spinOnce();
          loop_rate.sleep();
     }

     return 0;
}
