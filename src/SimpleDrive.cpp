/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "simple_drive_plugin/SimpleDrive.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SimpleDrive)

/////////////////////////////////////////////////
SimpleDrive::SimpleDrive()
{
}

/////////////////////////////////////////////////
SimpleDrive::~SimpleDrive()
{
}

/////////////////////////////////////////////////
void SimpleDrive::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Output the name of the model
  std::cout << std::endl
            << "SimpleDrive: The realsense_camera plugin is attach to model "
            << _model->GetName() << std::endl;

  // Store a pointer to the this model
  this->drive_rsModel = _model;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  ROS_INFO("SimpleDrive plugin loading.");

  this->drive_rosnode_ = new ros::NodeHandle("/r1");

  // Subscribe to the https://github.com/tuw-robotics/tuw_teleop/tree/master/tuw_keyboard2twist node
  drive_sub = drive_rosnode_->subscribe("cmd_vel",1000,&SimpleDrive::CmdVelCallback,this);
}

/////////////////////////////////////////////////
void SimpleDrive::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  geometry_msgs::Vector3 linear =  msg->linear;
  geometry_msgs::Vector3 angular = msg->angular;
  ROS_INFO("Linear Vel on r1/cmd_vel: %f, %f, %f",linear.x,linear.y,linear.z);
  ROS_INFO("Angular Vel on r1/cmd_vel: %f, %f, %f",angular.x,angular.y,angular.z);
  //ROS_INFO("I heard: I got a message on r1/cmd_vel");


  gazebo::math::Pose pose = this->drive_rsModel->GetWorldPose();
  gazebo::math::Quaternion quaternion = pose.rot;
  // Communative because they rotate around the same axis
  gazebo::math::Quaternion new_quaternion =  gazebo::math::Quaternion(angular.x,angular.y,angular.z)*pose.rot;
  //gazebo::math::Quaternion new_quaternion =  pose.rot*gazebo::math::Quaternion(angular.x,angular.y,angular.z);
  gazebo::math::Vector3 Pos = pose.pos;

  float x_new = Pos.x + linear.x*cos(new_quaternion.GetYaw());
  float y_new = Pos.y + linear.x*sin(new_quaternion.GetYaw());

  
  gazebo::math::Pose new_pose = gazebo::math::Pose(gazebo::math::Vector3(x_new,y_new,0.05),new_quaternion);


  //ignition::math::Vector3d gazebo_linear = ignition::math::Vector3d(linear.x,linear.y,linear.z);
  //ignition::math::Vector3d gazebo_angular = ignition::math::Vector3d(angular.x,angular.y,angular.z);

  // Whats the point of this ? Only rotates model around its own axis
  //this->rsModel->SetWorldTwist(gazebo_linear,gazebo_angular); 

  // Similar to TWIST
  /*gazebo::math::Pose new_pose = gazebo::math::Pose(gazebo_linear.X() + Pos.x,
                                                           gazebo_linear.Y() + Pos.y,
                                                           gazebo_linear.Z() + Pos.z,
                                                           quaternion.GetRoll() + angular.x,
                                                           quaternion.GetPitch() + angular.y,
                                                           quaternion.GetYaw() + angular.z); */

  this->drive_rsModel->SetWorldPose(new_pose);
  //this->drive_rsModel->SetWorldPose(gazebo::math::Pose(gazebo::math::Vector3(2,0,0.2),new_quaternion));
  
}

/////////////////////////////////////////////////
void SimpleDrive::OnUpdate()
{
    ros::spinOnce();
}
