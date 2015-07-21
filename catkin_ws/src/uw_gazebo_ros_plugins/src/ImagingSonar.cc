/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/*
 * Desc: Imaging Sonar plugin
 * Author: Nate Koenig mod by John Hsu rewrite by Kevin DeMarco
 */

#include <iostream>
#include "gazebo/physics/physics.hh"
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/Node.hh>

#include "ImagingSonar.hh"

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

#include <tf/tf.h>

#define EPSILON_DIFF 0.000001

using namespace gazebo;

using std::cout;
using std::endl;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(ImagingSonar)

/////////////////////////////////////////////////
ImagingSonar::ImagingSonar()
{
}

/////////////////////////////////////////////////
ImagingSonar::~ImagingSonar()
{
     this->parent_sensor_->GetLaserShape()->DisconnectNewLaserScans(
          this->newLaserScansConnection);
     this->newLaserScansConnection.reset();

     this->parent_sensor_.reset();
     this->world_.reset();
}

/////////////////////////////////////////////////
void ImagingSonar::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
     
     cout << "=======================================" << endl;
     cout << "Loading ImagingSonar Plugin." << endl;     
     
     // Get then name of the parent sensor
     this->parent_sensor_ =
          boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);

     if (!this->parent_sensor_)
          gzthrow("ImagingSonar requires a Ray Sensor as its parent");

     this->world_ = physics::get_world(this->parent_sensor_->GetWorldName());

     this->newLaserScansConnection =
          this->parent_sensor_->GetLaserShape()->ConnectNewLaserScans(
               boost::bind(&ImagingSonar::OnNewLaserScans, this));

     std::string worldName = _parent->GetWorldName();
     last_update_time_ = this->world_->GetSimTime();
     this->node_ = transport::NodePtr(new transport::Node());
     this->node_->Init(worldName);
     
     this->robot_namespace_ = "";
     if (_sdf->HasElement("robotNamespace"))
          this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

     if (!_sdf->HasElement("frameName"))
     {
          ROS_INFO("Block laser plugin missing <frameName>, defaults to /world");
          this->frame_name_ = "/world";
     }
     else
          this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

     if (!_sdf->HasElement("topicName"))
     {
          ROS_INFO("Block laser plugin missing <topicName>, defaults to /world");
          this->topic_name_ = "/world";
     }
     else
          this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();


     if (!_sdf->GetElement("updateRate"))
     {
          ROS_INFO("Block laser plugin missing <updateRate>, defaults to 0");
          this->update_rate_ = 0;
     }
     else
          this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
     // FIXME:  update the update_rate_

     // Make sure the ROS node for Gazebo has already been initialized
     if (!ros::isInitialized())
     {
          ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                           << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
          return;
     }

     this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
     
     // resolve tf prefix
     std::string prefix;
     this->rosnode_->getParam(std::string("tf_prefix"), prefix);
     this->frame_name_ = tf::resolve(prefix, this->frame_name_);

     //// set size of cloud message, starts at 0!! FIXME: not necessary
     //this->cloud_msg_.points.clear();
     //this->cloud_msg_.channels.clear();
     //this->cloud_msg_.channels.push_back(sensor_msgs::ChannelFloat32());

     if (this->topic_name_ != "")
     {
          // Custom Callback Queue
          ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud>(
               this->topic_name_,1,
               boost::bind( &ImagingSonar::LaserConnect,this),
               boost::bind( &ImagingSonar::LaserDisconnect,this), ros::VoidPtr(), &this->laser_queue_);
          this->pub_ = this->rosnode_->advertise(ao);
     }

     // sensor generation off by default
     this->parent_sensor_->SetActive(false);

     this->callback_laser_queue_thread_ = boost::thread( boost::bind( &ImagingSonar::LaserQueueThread,this ) );

     cout << "Finished Loading" << endl;
     cout << "=======================================" << endl;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void ImagingSonar::LaserConnect()
{
     this->laser_connect_count_++;
     this->parent_sensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void ImagingSonar::LaserDisconnect()
{
     this->laser_connect_count_--;

     if (this->laser_connect_count_ == 0)
          this->parent_sensor_->SetActive(false);
}

/////////////////////////////////////////////////
void ImagingSonar::OnNewLaserScans()
{
     if (this->topic_name_ != "") {
          common::Time sensor_update_time = this->parent_sensor_->GetLastUpdateTime();
          if (last_update_time_ < sensor_update_time) {
               //this->PutSonarImage(sensor_update_time);
               this->PutLaserData(sensor_update_time);
               last_update_time_ = sensor_update_time;
          }
     } else {
          cout << "ImagingSonar topic name not set" << endl;
     }
}

void ImagingSonar::PutSonarImage(common::Time &_updateTime)
{
     //cout << "New scan..." << endl;
     this->parent_sensor_->SetActive(false);
     
     math::Angle maxAngle = this->parent_sensor_->GetAngleMax();
     math::Angle minAngle = this->parent_sensor_->GetAngleMin();

     double maxRange = this->parent_sensor_->GetRangeMax();
     double minRange = this->parent_sensor_->GetRangeMin();
     int rayCount = this->parent_sensor_->GetRayCount();
     int rangeCount = this->parent_sensor_->GetRangeCount();

     int verticalRayCount = this->parent_sensor_->GetVerticalRayCount();
     int verticalRangeCount = this->parent_sensor_->GetVerticalRangeCount();
     math::Angle verticalMaxAngle = this->parent_sensor_->GetVerticalAngleMax();
     math::Angle verticalMinAngle = this->parent_sensor_->GetVerticalAngleMin();

     double yDiff = maxAngle.Radian() - minAngle.Radian();
     double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();
     
     ////
     /// Figure out how gazebo_ros_block_laser makes the point cloud
     /// interpolation?
     ////
     

     this->parent_sensor_->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void ImagingSonar::PutLaserData(common::Time &_updateTime)
{
     int i, hja, hjb;
     int j, vja, vjb;
     double vb, hb;
     int    j1, j2, j3, j4; // four corners indices
     double r1, r2, r3, r4, r; // four corner values + interpolated range
     double intensity;

     this->parent_sensor_->SetActive(false);

     math::Angle maxAngle = this->parent_sensor_->GetAngleMax();
     math::Angle minAngle = this->parent_sensor_->GetAngleMin();

     double maxRange = this->parent_sensor_->GetRangeMax();
     double minRange = this->parent_sensor_->GetRangeMin();
     int rayCount = this->parent_sensor_->GetRayCount();
     int rangeCount = this->parent_sensor_->GetRangeCount();

     int verticalRayCount = this->parent_sensor_->GetVerticalRayCount();
     int verticalRangeCount = this->parent_sensor_->GetVerticalRangeCount();
     math::Angle verticalMaxAngle = this->parent_sensor_->GetVerticalAngleMax();
     math::Angle verticalMinAngle = this->parent_sensor_->GetVerticalAngleMin();

     double yDiff = maxAngle.Radian() - minAngle.Radian();
     double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

     //cout << "================================" << endl;
     //cout << "maxAngle: " << maxAngle << endl;
     //cout << "minAngle: " << minAngle << endl;
     //cout << "maxRange: " << maxRange << endl;
     //cout << "minRange: " << minRange  << endl;
     //cout << "rayCount: " << rayCount  << endl;
     //cout << "rangeCount: " << rangeCount  << endl;
     //cout << "verticalRayCount: " << verticalRayCount  << endl;
     //cout << "verticalRangeCount: " << verticalRangeCount  << endl;
     //cout << "verticalMaxAngle: " << verticalMaxAngle  << endl;
     //cout << "verticalMinAngle: " << verticalMinAngle  << endl;
     //cout << "yDiff: " << yDiff  << endl;
     //cout << "pDiff: " << pDiff  << endl;

     // set size of cloud message everytime!
     //int r_size = rangeCount * verticalRangeCount;
     this->cloud_msg_.points.clear();
     this->cloud_msg_.channels.clear();
     this->cloud_msg_.channels.push_back(sensor_msgs::ChannelFloat32());

     /***************************************************************/
     /*                                                             */
     /*  point scan from laser                                      */
     /*                                                             */
     /***************************************************************/
     boost::mutex::scoped_lock sclock(this->lock);
     // Add Frame Name
     this->cloud_msg_.header.frame_id = this->frame_name_;
     this->cloud_msg_.header.stamp.sec = _updateTime.sec;
     this->cloud_msg_.header.stamp.nsec = _updateTime.nsec;

     for (j = 0; j<verticalRangeCount; j++)
     {
          // interpolating in vertical direction
          vb = (verticalRangeCount == 1) ? 0 : (double) j * (verticalRayCount - 1) / (verticalRangeCount - 1);
          vja = (int) floor(vb);
          vjb = std::min(vja + 1, verticalRayCount - 1);
          vb = vb - floor(vb); // fraction from min

          assert(vja >= 0 && vja < verticalRayCount);
          assert(vjb >= 0 && vjb < verticalRayCount);

          for (i = 0; i<rangeCount; i++)
          {
               // Interpolate the range readings from the rays in horizontal direction
               hb = (rangeCount == 1)? 0 : (double) i * (rayCount - 1) / (rangeCount - 1);
               hja = (int) floor(hb);
               hjb = std::min(hja + 1, rayCount - 1);
               hb = hb - floor(hb); // fraction from min

               assert(hja >= 0 && hja < rayCount);
               assert(hjb >= 0 && hjb < rayCount);

               // indices of 4 corners
               j1 = hja + vja * rayCount;
               j2 = hjb + vja * rayCount;
               j3 = hja + vjb * rayCount;
               j4 = hjb + vjb * rayCount;
               // range readings of 4 corners
               r1 = std::min(this->parent_sensor_->GetLaserShape()->GetRange(j1) , maxRange-minRange);
               r2 = std::min(this->parent_sensor_->GetLaserShape()->GetRange(j2) , maxRange-minRange);
               r3 = std::min(this->parent_sensor_->GetLaserShape()->GetRange(j3) , maxRange-minRange);
               r4 = std::min(this->parent_sensor_->GetLaserShape()->GetRange(j4) , maxRange-minRange);      

               // Range is linear interpolation if values are close,
               // and min if they are very different
               r = (1-vb)*((1 - hb) * r1 + hb * r2)
                    +   vb *((1 - hb) * r3 + hb * r4);      

               // Intensity is averaged
               intensity = 0.25*(this->parent_sensor_->GetLaserShape()->GetRetro(j1) +
                                 this->parent_sensor_->GetLaserShape()->GetRetro(j2) +
                                 this->parent_sensor_->GetLaserShape()->GetRetro(j3) +
                                 this->parent_sensor_->GetLaserShape()->GetRetro(j4));                     

               // std::cout << " block debug "
               //           << "  ij("<<i<<","<<j<<")"
               //           << "  j1234("<<j1<<","<<j2<<","<<j3<<","<<j4<<")"
               //           << "  r1234("<<r1<<","<<r2<<","<<r3<<","<<r4<<")"
               //           << std::endl;

               // get angles of ray to get xyz for point
               double yAngle = 0.5*(hja+hjb) * yDiff / (rayCount -1) + minAngle.Radian();
               double pAngle = 0.5*(vja+vjb) * pDiff / (verticalRayCount -1) + verticalMinAngle.Radian();

               //if (j == 0 && i == 0) {
               //     cout << "========================" << endl;
               //     cout << "Retro1: " << this->parent_sensor_->GetLaserShape()->GetRetro(j1) << endl;
               //     cout << "Retro2: " << this->parent_sensor_->GetLaserShape()->GetRetro(j2) << endl;
               //     cout << "Retro3: " << this->parent_sensor_->GetLaserShape()->GetRetro(j3) << endl;
               //     cout << "Retro4: " << this->parent_sensor_->GetLaserShape()->GetRetro(j4) << endl;               
               //     cout << "vb: " << vb << endl;
               //     cout << "hb: " << hb << endl;
               //     cout << "hja: " << hja << endl;
               //     cout << "hjb: " << hjb << endl;
               //     cout << "vja: " << vja << endl;
               //     cout << "vjb: " << vjb << endl;
               //     cout << "j1: " << j1 << endl;
               //     cout << "j2: " << j2 << endl;
               //     cout << "j3: " << j3 << endl;
               //     cout << "j4: " << j4 << endl;
               //     cout << "r1: " << r1 << endl;
               //     cout << "r2: " << r2 << endl;
               //     cout << "r3: " << r3 << endl;
               //     cout << "r4: " << r4 << endl;
               //     cout << "r: " << r << endl;           
               //     cout << "intensity: " << intensity << endl;
               //     cout << "Retro(j1): " << this->parent_sensor_->GetLaserShape()->GetRetro(j1) << endl;
               //     cout << "Retro(j2): " << this->parent_sensor_->GetLaserShape()->GetRetro(j2) << endl;
               //     cout << "Retro(j3): " << this->parent_sensor_->GetLaserShape()->GetRetro(j3) << endl;
               //     cout << "Retro(j4): " << this->parent_sensor_->GetLaserShape()->GetRetro(j4) << endl;
               //     cout << "yAngle: " << yAngle << endl;
               //     cout << "pAngle: " << pAngle << endl;           
               //}

               /***************************************************************/
               /*                                                             */
               /*  point scan from laser                                      */
               /*                                                             */
               /***************************************************************/
               //compare 2 doubles
               double diffRange = maxRange - minRange;
               double diff  = diffRange - r;
               //if (fabs(diff) < EPSILON_DIFF)
               //{
               //  // no noise if at max range
               //  geometry_msgs::Point32 point;
               //  //pAngle is rotated by yAngle:
               //  point.x = r * cos(pAngle) * cos(yAngle);
               //  point.y = r * cos(pAngle) * sin(yAngle);
               //  point.z = -r * sin(pAngle);
               //
               //  this->cloud_msg_.points.push_back(point); 
               //} 
               //else 
               { 
                    geometry_msgs::Point32 point;
                    //pAngle is rotated by yAngle:
                    //point.x = r * cos(pAngle) * cos(yAngle) + this->GaussianKernel(0,this->gaussian_noise_);
                    //point.y = r * cos(pAngle) * sin(yAngle) + this->GaussianKernel(0,this->gaussian_noise_);
                    //point.z = -r * sin(pAngle) + this->GaussianKernel(0,this->gaussian_noise_);
        
                    point.x = r * cos(pAngle) * cos(yAngle);
                    point.y = r * cos(pAngle) * sin(yAngle);
                    point.z = r * sin(pAngle);                        
        
                    this->cloud_msg_.points.push_back(point); 
               } // only 1 channel 

               //this->cloud_msg_.channels[0].values.push_back(intensity + this->GaussianKernel(0,this->gaussian_noise_)) ;
               this->cloud_msg_.channels[0].values.push_back(intensity) ;
          }
     }
     this->parent_sensor_->SetActive(true);

     // send data out via ros message
     this->pub_.publish(this->cloud_msg_);
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void ImagingSonar::LaserQueueThread()
{
     static const double timeout = 0.01;

     while (this->rosnode_->ok())
     {
          this->laser_queue_.callAvailable(ros::WallDuration(timeout));
     }
}

void ImagingSonar::OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
{
     this->sim_time_  = msgs::Convert( _msg->sim_time() );

     math::Pose pose;
     pose.pos.x = 0.5*sin(0.01*this->sim_time_.Double());
     gzdbg << "plugin simTime [" << this->sim_time_.Double() << "] update pose [" << pose.pos.x << "]\n";
}
