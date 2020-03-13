/**---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
 * This file is part of the Neurorobotics Platform software
 * Copyright (C) 2014,2015,2016,2017 Human Brain Project
 * https://www.humanbrainproject.eu
 *
 * The Human Brain Project is a European Commission funded project
 * in the frame of the Horizon2020 FET Flagship plan.
 * http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * ---LICENSE-END**/
#ifndef GENERIC_CONTROLLER_PLUGIN_H
#define GENERIC_CONTROLLER_PLUGIN_H

#include <vector>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <generic_controller_plugin/SetPIDParameters.h>
#include <generic_controller_plugin/JointProperties.h>

#include <gazebo_msgs/SetJointStates.h>
#include <gazebo_msgs/GetJointStates.h>

#include <ros/ros.h>

#include <algorithm>
#include <mutex>

namespace gazebo
{

typedef std::map<std::string, physics::JointPtr> JointMap;

using namespace generic_controller_plugin;

class GenericControlPlugin : public ModelPlugin
{

public:

  GenericControlPlugin();
  ~GenericControlPlugin();

  // Load the plugin and initilize all controllers
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

  // Simulation update callback function
  void OnUpdate(const common::UpdateInfo & /*_info*/);

private:

  // check if controller for current joint is specified in SDF and return pointer to sdf element
  bool existsControllerSDF(sdf::ElementPtr &sdf_ctrl_def, const sdf::ElementPtr &sdf,
                           const physics::JointPtr &joint);

  // check if visual properties (for client-side animation of models) exist, and return a pointer to the SDF element
  bool existsVisualSDF(sdf::ElementPtr &sdf_visual_def, const sdf::ElementPtr& sdf,
                       const physics::JointPtr& joint);

  // get PID controller values from SDF
  common::PID getControllerPID(const sdf::ElementPtr &sdf_ctrl_def);

  // get controller type from SDF
  std::string getControllerType(const sdf::ElementPtr &sdf_ctrl_def);

  // Method for creating a position controller for a given joint
  void createPositionController(const physics::JointPtr &joint, const common::PID &pid_param);

  // Method for creating a velocity controller for a given joint
  void createVelocityController(const physics::JointPtr &joint, const common::PID &pid_param);

  // Generic position command callback function (ROS topic)
  void positionCB(const std_msgs::Float64::ConstPtr &msg, const physics::JointPtr &joint);

  // Generic velocity command callback function (ROS topic)
  void velocityCB(const std_msgs::Float64::ConstPtr &msg, const physics::JointPtr &joint);

  // Generic get command callback function (ROS service), used by getPositionServiceCB and getVelocityServiceCB
  template<class T>
  bool getGeneralServiceCB(const gazebo_msgs::GetJointStates::Request &req, gazebo_msgs::GetJointStates::Response &res, const physics::JointPtr &joint, const T &value_map);

  // Generic position get command callback function (ROS service)
  bool getPositionServiceCB(const gazebo_msgs::GetJointStates::Request &req, gazebo_msgs::GetJointStates::Response &res, const physics::JointPtr &joint);

  // Generic velocity get command callback function (ROS service)
  bool getVelocityServiceCB(const gazebo_msgs::GetJointStates::Request &req, gazebo_msgs::GetJointStates::Response &res, const physics::JointPtr &joint);

  // Generic position set command callback function (ROS service)
  bool setPositionServiceCB(const gazebo_msgs::SetJointStates::Request &req, gazebo_msgs::SetJointStates::Response &res, const physics::JointPtr &joint);

  // Generic velocity set command callback function (ROS service)
  bool setVelocityServiceCB(const gazebo_msgs::SetJointStates::Request &req, gazebo_msgs::SetJointStates::Response &res, const physics::JointPtr &joint);

  // Generic PID parameter setter callback function (ROS service)
  bool setPIDParametersCB(SetPIDParameters::Request &req,
                          SetPIDParameters::Response &res);
  
  // Joint properties getter callback function (ROS service)
  bool getJointPropertiesCB(JointProperties::Request &req,
                             JointProperties::Response &res);

  // ROS node handle
  ros::NodeHandle m_nh;

  // Pointer to the model
  physics::ModelPtr m_model;

  // Pointer to joint controllers
  physics::JointControllerPtr m_joint_controller;

  // Map of joint pointers
  JointMap m_joints;

  // Pointer to the update event connection
  event::ConnectionPtr m_updateConnection;

  // ROS subscriber for joint control values
  std::vector<ros::Subscriber> m_pos_sub_vec;
  std::vector<ros::Subscriber> m_vel_sub_vec;

  // ROS service for joint control values
  std::vector<ros::ServiceServer> m_pos_service_vec;
  std::vector<ros::ServiceServer> m_vel_service_vec;

  // Maps for joint names and rotation axes (visual properties for client-side animation)
  std::map<std::string, std::string> m_joint_name_mappings;
  std::map<std::string, geometry_msgs::Vector3> m_joint_axis_mappings;

  /// \brief keep track of controller update sim-time.
  private: gazebo::common::Time lastControllerUpdateTime;

  /// \brief Controller update mutex.
  private: std::mutex mutex;

  // ROS joint state publisher
  private: ros::Publisher m_joint_state_pub;
  private: sensor_msgs::JointState m_js;
  private: ros::Publisher m_joint_accel_pub;
  private: std_msgs::Float32MultiArray m_ja;
  // ROS joint controller parameter setter service
  private: ros::ServiceServer m_setPIDParameterService;
  
  // ROS joint properties getter service (joint names, lower/upper limits)
  private: ros::ServiceServer m_jointPropertiesService;
};

template<class T>
bool GenericControlPlugin::getGeneralServiceCB(const gazebo_msgs::GetJointStates::Request &req, gazebo_msgs::GetJointStates::Response &res, const physics::JointPtr &joint, const T &value_map)
{
  auto joint_iterator = value_map.find(joint->GetScopedName());
  if(joint_iterator == value_map.end())
  {
    res.success = false;
    res.status_message = "Could not find joint " + joint->GetScopedName();
  }
  else
  {
    res.success = true;
    res.status_message = "";
    res.value.resize(1, joint_iterator->second);
  }

  return res.success;
}

} // namespace gazebo

#endif
