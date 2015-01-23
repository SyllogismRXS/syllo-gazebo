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
 * Desc: Ray Plugin
 * Author: Nate Koenig mod by John Hsu
 */

#ifndef _GAZEBO_IMAGING_SONAR_HH_
#define _GAZEBO_IMAGING_SONAR_HH_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "gazebo/util/system.hh"

#include <sensor_msgs/PointCloud.h>

namespace gazebo
{
     /// \brief A Ray Sensor Plugin
     class GAZEBO_VISIBLE ImagingSonar : public SensorPlugin
     {
          /// \brief Constructor
     public: ImagingSonar();

          /// \brief Destructor
     public: virtual ~ImagingSonar();

          /// \brief Update callback
     public: virtual void OnNewLaserScans();

          /// \brief Load the plugin
          /// \param take in SDF root element
     public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

          /// \brief Pointer to parent
     protected: physics::WorldPtr world_;

          /// \brief The parent sensor
     private: sensors::RaySensorPtr parent_sensor_;

          /// \brief The connection tied to ImagingSonar::OnNewLaserScans()
     private: event::ConnectionPtr newLaserScansConnection;

          /// \brief pointer to ros node
     private: ros::NodeHandle* rosnode_;
     private: ros::Publisher pub_;

          // \brief ros message
     private: sensor_msgs::PointCloud cloud_msg_;

          /// \brief topic name
    private: std::string topic_name_;
      /// \brief frame transform name, should match link name
    private: std::string frame_name_;
          
          /// update rate of this sensor
     private: double update_rate_;
          
          /// \brief for setting ROS name space
     private: std::string robot_namespace_;
     private: common::Time last_update_time_;
          
          /// \brief Gaussian noise
          //private: double gaussian_noise_;

          /// \brief Gaussian noise generator
          //private: double GaussianKernel(double mu,double sigma);
          
          /// \brief A mutex to lock access to fields that are used in message callbacks
     private: boost::mutex lock;

     private: int laser_connect_count_;
     private: void LaserConnect();
     private: void LaserDisconnect();

     protected: void PutSonarImage(common::Time &_updateTime);
     protected: void PutLaserData(common::Time &_updateTime);

          // Custom Callback Queue
     private: ros::CallbackQueue laser_queue_;
     private: void LaserQueueThread();
     private: boost::thread callback_laser_queue_thread_;

          // subscribe to world stats
     private: transport::NodePtr node_;
     private: common::Time sim_time_;
     public: void OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg);
     };
}
#endif
