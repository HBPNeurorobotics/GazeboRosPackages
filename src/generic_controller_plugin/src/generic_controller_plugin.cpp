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
/*
 * Desc: Gazebo plugin providing generic controllers for robot joints
 * This plugin provides ROS topics to control single joints of the robot. The controlled joints can be specified in the SDF as plugin tags
 * Author: Lars Pfotzer
 */

#include "generic_controller_plugin.h"

#include <boost/bind.hpp>
#include <sensor_msgs/JointState.h>
#include <ros/time.h>

namespace gazebo
{

GenericControlPlugin::GenericControlPlugin()
{
  m_nh = ros::NodeHandle();
}

GenericControlPlugin::~GenericControlPlugin()
{
  m_nh.shutdown();
}

void GenericControlPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  // Store the pointer to the model
  m_model = parent;
  m_joint_controller = m_model->GetJointController();
  if (m_joint_controller) // Null if there are no joints
    m_joints = m_joint_controller->GetJoints();

  //sdf::ElementPtr sdf->GetElement("");
  ROS_INFO("sdf name %s, sdf description %s", sdf->GetName().c_str(), sdf->GetDescription().c_str());

  for (JointMap::iterator joint_iter = m_joints.begin(); joint_iter != m_joints.end(); ++joint_iter)
  {
    physics::JointPtr joint = joint_iter->second;

    sdf::ElementPtr sdf_ctrl_def;

    // check, if controller for current joint was specified in the SDF and return sdf element pointer to controller
    if (existsControllerSDF(sdf_ctrl_def, sdf, joint))
    {
      // get controller parameter from sdf file
      common::PID ctrl_pid = getControllerPID(sdf_ctrl_def);
      std::string ctrl_type = getControllerType(sdf_ctrl_def);

      // create controller regarding the specified controller type
      if (ctrl_type == "position")
      {
        createPositionController(joint, ctrl_pid);
      }
      else if (ctrl_type == "velocity")
      {
        createVelocityController(joint, ctrl_pid);
      }
    }

    sdf::ElementPtr sdf_visual_def;
    if (existsVisualSDF(sdf_visual_def, sdf, joint))
    {
      sdf::ElementPtr joint_name_element = sdf_visual_def->GetElement("joint_name");
      sdf::ElementPtr joint_axis_element = sdf_visual_def->GetElement("joint_axis");

      if (joint_name_element != NULL)
      {
        this->m_joint_name_mappings[joint->GetName()] = sdf_visual_def->Get<std::string>();

        joint->SetMappedName(joint_name_element->Get<std::string>());
      }

      if (joint_axis_element != NULL)
      {
#if SDF_MAJOR_VERSION > 3 && GAZEBO_MAJOR_VERSION > 6
        ignition::math::Vector3d joint_axis = joint_axis_element->Get<ignition::math::Vector3d>();
        geometry_msgs::Vector3 joint_axis_ros;
        joint_axis_ros.x = joint_axis.X();
        joint_axis_ros.y = joint_axis.Y();
        joint_axis_ros.z = joint_axis.Z();

        this->m_joint_axis_mappings[joint->GetName()] = joint_axis_ros;
        joint->SetMappedRotationAxis(joint_axis);
#else
        sdf::Vector3 joint_axis = joint_axis_element->GetValueVector3();
        geometry_msgs::Vector3 joint_axis_ros;
        joint_axis_ros.x = joint_axis.x;
        joint_axis_ros.y = joint_axis.y;
        joint_axis_ros.z = joint_axis.z;

        this->m_joint_axis_mappings[joint->GetName()] = joint_axis_ros;
        ignition::math::Vector3d rotation_axis(joint_axis.x, joint_axis.y, joint_axis.z);
        joint->SetMappedRotationAxis(rotation_axis);
#endif
      }

      // Presence of name mapping triggers push-notification of joint state
      if (joint_name_element != NULL)
        joint->SetPushState(true);
    }
  }

  // Controller time control.
  this->lastControllerUpdateTime = this->m_model->GetWorld()->SimTime();

  int numJoints = m_joints.size();
  m_js.header.stamp.sec = this->lastControllerUpdateTime.sec;
  m_js.header.stamp.nsec = this->lastControllerUpdateTime.nsec;
  m_js.name.resize ( numJoints );
  m_js.position.resize ( numJoints );
  m_js.velocity.resize ( numJoints );
  m_js.effort.resize ( numJoints );

  // Listen to the update event. This event is broadcast every simulation iteration.
  m_updateConnection = event::Events::ConnectBeforePhysicsUpdate(boost::bind(&GenericControlPlugin::OnUpdate, this, _1));

  const std::string joint_states_topic_name = m_model->GetName() + "/" + "joint_states";

  this->m_joint_state_pub = m_nh.advertise<sensor_msgs::JointState>( joint_states_topic_name, 10 );

  m_setPIDParameterService = m_nh.advertiseService<
    SetPIDParameters::Request,
    SetPIDParameters::Response>(
      m_model->GetName() + "/set_pid_parameters",
      boost::bind(&GenericControlPlugin::setPIDParametersCB, this, _1, _2)
      
  );
    
  m_jointPropertiesService = m_nh.advertiseService<
    JointProperties::Request,
    JointProperties::Response>(
      m_model->GetName() + "/joint_properties",
      boost::bind(&GenericControlPlugin::getJointPropertiesCB, this, _1, _2)
  );

  m_ja.layout.dim.resize(1);
  m_ja.layout.dim[0].label = "accelerations";
  m_ja.layout.dim[0].size  = numJoints;
  m_ja.layout.dim[0].stride = 1;
  m_ja.data.resize(numJoints);

  const std::string joint_accel_topic_name = m_model->GetName() + "/" + "joint_accel";

  this->m_joint_accel_pub = m_nh.advertise<std_msgs::Float32MultiArray>( joint_accel_topic_name, 10 );
}

// Called by the world update start event
void GenericControlPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  gazebo::common::Time curTime = this->m_model->GetWorld()->SimTime();

  if (curTime > this->lastControllerUpdateTime)
  {
    m_js.header.stamp.sec = curTime.sec;
    m_js.header.stamp.nsec = curTime.nsec;
    // Update the control surfaces and publish the new state.
    int curr_ind = 0;
    for (JointMap::iterator joint_iter = m_joints.begin(); joint_iter != m_joints.end(); ++joint_iter, ++curr_ind)
    {
      physics::JointPtr joint = joint_iter->second;
      m_js.name[curr_ind] = joint->GetName();
      m_js.position[curr_ind] = joint->Position();
      m_js.velocity[curr_ind] = joint->GetVelocity(0);
      m_js.effort[curr_ind] = joint->GetForce(0);
      m_ja.data[curr_ind] = joint->GetAcceleration(0);
    }

    m_joint_state_pub.publish ( m_js );
    m_joint_accel_pub.publish ( m_ja );
  }
  this->lastControllerUpdateTime = curTime;

}

///////////////////////////////////////// SDF parser functions ////////////////////////////////////////////

bool GenericControlPlugin::existsControllerSDF(sdf::ElementPtr &sdf_ctrl_def, const sdf::ElementPtr &sdf,
                                               const physics::JointPtr &joint)
{
  sdf::ElementPtr sdf_ctrl = sdf->GetElement("controller");
  while (sdf_ctrl != NULL)
  {
    sdf::ParamPtr joint_name_attr = sdf_ctrl->GetAttribute("joint_name");
    if (joint_name_attr != NULL)
    {
      std::string joint_name = joint_name_attr->GetAsString();
      if (joint_name == joint->GetName())
      {
        ROS_INFO("Found controller for joint %s", joint_name.c_str());
        sdf_ctrl_def = sdf_ctrl;
        return true;
      }
      else
      {
        // find next controller
        sdf_ctrl = sdf_ctrl->GetNextElement("controller");
      }
    }
    else
    {
      ROS_WARN("Attribute 'joint_name' is not available for current controller.");
      return false;
    }
  }

  ROS_WARN("No controller for joint %s found", joint->GetName().c_str());
  return false;
}

bool GenericControlPlugin::existsVisualSDF(sdf::ElementPtr& sdf_visual_def, const sdf::ElementPtr& sdf,
                                              const physics::JointPtr& joint)
{
  sdf::ElementPtr sdf_visual = sdf->GetElement("visual");
  while (sdf_visual != NULL)
  {
    sdf::ParamPtr joint_name_param = sdf_visual->GetAttribute("joint_name");
    if (joint_name_param != NULL)
    {
      std::string joint_name = joint_name_param->GetAsString();
      if (joint_name.compare(joint->GetName()) == 0)
      {
        ROS_INFO("Found visual properties for joint %s", joint_name.c_str());
        sdf_visual_def = sdf_visual;
        return true;
      }
      else
      {
        sdf_visual = sdf_visual->GetNextElement("visual");
      }
    }
    else
    {
      ROS_WARN("Attribute 'joint_name' is not available for current joint definition.");
      return false;
    }
  }

  return false;
}

common::PID GenericControlPlugin::getControllerPID(const sdf::ElementPtr &sdf_ctrl_def)
{
  common::PID pid_param;

  if (sdf_ctrl_def != NULL)
  {
    sdf::ElementPtr elem_pid = sdf_ctrl_def->GetElement("pid");
    if (elem_pid != NULL)
    {
#if SDF_MAJOR_VERSION > 3
      ignition::math::Vector3d pid_values = elem_pid->Get<ignition::math::Vector3d>();
      ROS_INFO("Controller PID values p=%f, i=%f, d=%f", pid_values.X(), pid_values.Y(), pid_values.Z());
      pid_param = common::PID(pid_values.X(), pid_values.Y(), pid_values.Z());
#else
      sdf::Vector3 pid_values = elem_pid->Get<sdf::Vector3>();
      ROS_INFO("Controller PID values p=%f, i=%f, d=%f", pid_values.x, pid_values.y, pid_values.z);
      pid_param = common::PID(pid_values.x, pid_values.y, pid_values.z);
#endif
      return pid_param;
    }
  }

  ROS_WARN("Could not find controller PID parameter in SDF file: Using default values.");
  pid_param = common::PID(1.0, 0.1, 0.01);
  return pid_param;
}

std::string GenericControlPlugin::getControllerType(const sdf::ElementPtr &sdf_ctrl_def)
{
  std::string ctrl_type = "";

  if (sdf_ctrl_def != NULL)
  {
    sdf::ElementPtr elem_type = sdf_ctrl_def->GetElement("type");
    if (elem_type != NULL)
    {
      ctrl_type = elem_type->Get<std::string>();
      ROS_INFO("Controller has type %s", ctrl_type.c_str());
      return ctrl_type;
    }
  }

  ROS_WARN("Could not find controller type in SDF file.");
  return ctrl_type;
}

//////////////////////////////////////// Controller construction //////////////////////////////////////////

void GenericControlPlugin::createPositionController(const physics::JointPtr &joint, const common::PID &pid_param)
{
  // generate joint topic name using the model name as prefix
  std::string name_prefix = m_model->GetName() + "/" + joint->GetName();
  replace(name_prefix.begin(), name_prefix.end(), ':', '_');

  const std::string topic_name = name_prefix + "/cmd_pos";
  const std::string set_service_name = name_prefix + "/set_target";
  const std::string get_service_name = name_prefix + "/get_target";

  // Add ROS topic for position control
  m_pos_sub_vec.push_back(m_nh.subscribe<std_msgs::Float64>(topic_name, 1,
                                                            boost::bind(&GenericControlPlugin::positionCB, this, _1, joint)));

  // Add ROS service for position control
  m_pos_service_vec.push_back(m_nh.advertiseService<gazebo_msgs::GetJointStates::Request, gazebo_msgs::GetJointStates::Response>(get_service_name,
                                                                                 boost::bind(&GenericControlPlugin::getPositionServiceCB, this, _1, _2, joint)));

  m_pos_service_vec.push_back(m_nh.advertiseService<gazebo_msgs::SetJointStates::Request, gazebo_msgs::SetJointStates::Response>(set_service_name,
                                                                                 boost::bind(&GenericControlPlugin::setPositionServiceCB, this, _1, _2, joint)));

  // Store information of Actuator in Model class
  m_model->SaveControllerActuatorRosTopics(topic_name, "std_msgs/Float64");

  // Create PID parameter for position controller
  m_joint_controller->SetPositionPID(joint->GetScopedName(), pid_param);

  // Initialize controller with zero position
  m_joint_controller->SetPositionTarget(joint->GetScopedName(), 0.0);

  ROS_INFO("Added new position controller for joint %s", joint->GetName().c_str());
}

void GenericControlPlugin::createVelocityController(const physics::JointPtr &joint, const common::PID &pid_param)
{
  // generate joint topic name using the model name as prefix
  std::string name_prefix = m_model->GetName() + "/" + joint->GetName();
  replace(name_prefix.begin(), name_prefix.end(), ':', '_');

  const std::string topic_name = name_prefix + "/cmd_pos";
  const std::string set_service_name = name_prefix + "/set_target";
  const std::string get_service_name = name_prefix + "/get_target";

  // Add ROS topic for velocity control
  m_vel_sub_vec.push_back(m_nh.subscribe<std_msgs::Float64>(topic_name, 1,
                                                            boost::bind(&GenericControlPlugin::velocityCB, this, _1, joint)));

  // Add ROS service for position control
  m_vel_service_vec.push_back(m_nh.advertiseService<gazebo_msgs::GetJointStates::Request, gazebo_msgs::GetJointStates::Response>(get_service_name,
                                                                                 boost::bind(&GenericControlPlugin::getVelocityServiceCB, this, _1, _2, joint)));

  m_vel_service_vec.push_back(m_nh.advertiseService<gazebo_msgs::SetJointStates::Request, gazebo_msgs::SetJointStates::Response>(set_service_name,
                                                                                 boost::bind(&GenericControlPlugin::setVelocityServiceCB, this, _1, _2, joint)));

  // Store information of Actuator in Model class
  m_model->SaveControllerActuatorRosTopics(topic_name, "std_msgs/Float64");

  // Create PID parameter for velocity controller
  m_joint_controller->SetVelocityPID(joint->GetScopedName(), pid_param);

  // Initialize controller with zero velocity
  m_joint_controller->SetVelocityTarget(joint->GetScopedName(), 0.0);

  ROS_INFO("Added new velocity controller for joint %s", joint->GetName().c_str());
}

//////////////////////////////////////// ROS topic callback functions //////////////////////////////////////////

void GenericControlPlugin::positionCB(const std_msgs::Float64::ConstPtr &msg, const physics::JointPtr &joint)
{
  ROS_DEBUG("positionCB called! Joint name = %s, joint pos = %f", joint->GetName().c_str(), msg->data);

  double angle_rad(msg->data);
  m_joint_controller->SetPositionTarget(joint->GetScopedName(), angle_rad);
  //pid.SetCmd(angle_rad);
}

void GenericControlPlugin::velocityCB(const std_msgs::Float64::ConstPtr &msg, const physics::JointPtr &joint)
{
  ROS_DEBUG("velocityCB called! Joint name = %s, joint vel = %f", joint->GetName().c_str(), msg->data);
  double velocity_m_per_sec(msg->data);
  m_joint_controller->SetVelocityTarget(joint->GetScopedName(), velocity_m_per_sec);
  //pid.SetCmd(velocity_m_per_sec);
}

bool GenericControlPlugin::getPositionServiceCB(const gazebo_msgs::GetJointStates::Request &req, gazebo_msgs::GetJointStates::Response &res, const physics::JointPtr &joint)
{
  return getGeneralServiceCB(req, res, joint, m_joint_controller->GetPositions());
}

bool GenericControlPlugin::getVelocityServiceCB(const gazebo_msgs::GetJointStates::Request &req, gazebo_msgs::GetJointStates::Response &res, const physics::JointPtr &joint)
{
  return getGeneralServiceCB(req, res, joint, m_joint_controller->GetVelocities());
}

bool GenericControlPlugin::setPositionServiceCB(const gazebo_msgs::SetJointStates::Request &req, gazebo_msgs::SetJointStates::Response &res, const physics::JointPtr &joint)
{
  res.success = m_joint_controller->SetPositionTarget(joint->GetScopedName(), req.value.at(0));
  if(!res.success)
    res.status_message = "Could not set position target for joint " + joint->GetScopedName();

  return res.success;
}

bool GenericControlPlugin::setVelocityServiceCB(const gazebo_msgs::SetJointStates::Request &req, gazebo_msgs::SetJointStates::Response &res, const physics::JointPtr &joint)
{
  res.success = m_joint_controller->SetVelocityTarget(joint->GetScopedName(), req.value.at(0));
  if(!res.success)
    res.status_message = "Could not set velocity target for joint " + joint->GetScopedName();

  return res.success;
}

bool GenericControlPlugin::setPIDParametersCB(SetPIDParameters::Request &req,
                                              SetPIDParameters::Response &res)
{
  std::string joint_name = req.joint;
  double kp = req.kp,
         ki = req.ki,
         kd = req.kd;
  ROS_DEBUG("setPIDParametersCB called! Joint name = %s, kp = %f, ki = %f, kd = %f", joint_name.c_str(), kp, ki, kd);

  JointMap::iterator it = m_joints.find(joint_name);
  if (it == m_joints.end())
  {
    res.success = false;
    return false;
  }
  else
  {
    // We set same parameters for position and velocity PID. That is because
    // we do not know which kind of controller is used. At the time of this
    // writing this just puts the parameters in a map of the controller manager.
    // Parameters are obtained from there when the force is calculated.
    const std::string joint_id = it->second->GetScopedName();
    m_joint_controller->SetPositionPID(joint_id, common::PID(kp, ki, kd));
    m_joint_controller->SetVelocityPID(joint_id, common::PID(kp, ki, kd));
    res.success = true;
    return true;
  }
}

bool GenericControlPlugin::getJointPropertiesCB(JointProperties::Request &req,
                             JointProperties::Response &res)
{
  if (m_joints.size() > 0)
  {
    res.joint.resize(m_joints.size());
    res.lower_limit.resize(m_joints.size());
    res.upper_limit.resize(m_joints.size());
    int jointIdx = 0;
    for (JointMap::iterator joint_iter = m_joints.begin(); joint_iter != m_joints.end(); ++joint_iter)
    {
        physics::JointPtr joint = joint_iter->second;
        res.joint[jointIdx] = joint->GetName();
        
        // This only gives you the limits of the first joint axis/degree of freedom!
        res.lower_limit[jointIdx] = joint->LowerLimit(0);
        res.upper_limit[jointIdx] = joint->UpperLimit(0);
        jointIdx++;
    }
  }
  return true;
}
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GenericControlPlugin)

} // namespace gazebo
