#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

using namespace std;

namespace gazebo
{  
  class ROSController_iCub : public ModelPlugin
  {
  public: 
    void callbackpos(const std_msgs::Float64::ConstPtr &msg, const std::string &jointId) {
      double command = msg->data / this->DegOrRad;
      if (jointId == this->model->GetName() + "::eye_version")
      {
          gazebo::physics::JointPtr left_eye, right_eye;
          left_eye = this->joints.at(this->model->GetName() + "::left_eye_pan");
          right_eye = this->joints.at(this->model->GetName() + "::right_eye_pan");
          double vg = left_eye->GetAngle(0).Radian() - right_eye->GetAngle(0).Radian();
          double left_pan_pos = command + (vg / 2.0);
          double right_pan_pos = command - (vg / 2.0);
          this->jointControl->SetPositionTarget(this->model->GetName() + "::left_eye_pan", left_pan_pos);
          this->jointControl->SetPositionTarget(this->model->GetName() + "::right_eye_pan", right_pan_pos);
      }
      else if (jointId == "iCub::eye_vergence")
      {
          gazebo::physics::JointPtr left_eye, right_eye;
          left_eye = this->joints.at(this->model->GetName() + "::left_eye_pan");
          right_eye = this->joints.at(this->model->GetName() + "::right_eye_pan");
          double vs = (left_eye->GetAngle(0).Radian() + right_eye->GetAngle(0).Radian()) / 2.0;
          double left_pan_pos = vs + (command / 2.0);
          double right_pan_pos = vs - (command / 2.0);
          this->jointControl->SetPositionTarget(this->model->GetName() + "::left_eye_pan", left_pan_pos);
          this->jointControl->SetPositionTarget(this->model->GetName() + "::right_eye_pan", right_pan_pos);
      }
      else
        this->jointControl->SetPositionTarget(jointId, command);
    }
    void callbackvel(const std_msgs::Float64::ConstPtr &msg, const std::string &jointId) {
      double command = msg->data / this->DegOrRad;
      if (jointId == this->model->GetName() + "::eye_version")
      {
          gazebo::physics::JointPtr left_eye, right_eye;
          left_eye = this->joints.at(this->model->GetName() + "::left_eye_pan");
          right_eye = this->joints.at(this->model->GetName() + "::right_eye_pan");
          double vg = left_eye->GetVelocity(0) - right_eye->GetVelocity(0);
          double left_pan_pos = command + (vg / 2.0);
          double right_pan_pos = command - (vg / 2.0);
          this->jointControl->SetVelocityTarget(this->model->GetName() + "::left_eye_pan", left_pan_pos);
          this->jointControl->SetVelocityTarget(this->model->GetName() + "::right_eye_pan", right_pan_pos);
      }
      else if (jointId == this->model->GetName() + "::eye_vergence")
      {
          gazebo::physics::JointPtr left_eye, right_eye;
          left_eye = this->joints.at(this->model->GetName() + "::left_eye_pan");
          right_eye = this->joints.at(this->model->GetName() + "::right_eye_pan");
          double vs = (left_eye->GetVelocity(0) + right_eye->GetVelocity(0)) / 2.0;
          double left_pan_pos = vs + (command / 2.0);
          double right_pan_pos = vs - (command / 2.0);
          this->jointControl->SetVelocityTarget(this->model->GetName() + "::left_eye_pan", left_pan_pos);
          this->jointControl->SetVelocityTarget(this->model->GetName() + "::right_eye_pan", right_pan_pos);
      }
      else
        this->jointControl->SetVelocityTarget(jointId, command);
    }
    
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      string argument;

      if (!_sdf->HasElement("degree"))
      {
        std::cout << "Missing parameter <Degree> in ROSController_iCub, default to radiants" << std::endl;
        this->DegOrRad = 1;
      }
      else if (_sdf->Get<std::string>("degree") == "true")
      {
        this->DegOrRad = 180 / 3.141592653589793;
      }
      else if (_sdf->Get<std::string>("degree") == "false")
      {
        this->DegOrRad = 1;
      }
      else
      {
        std::cout << "Wrong value for the parameter <Degree> in ROSController_iCub, default to radiants" << std::endl;
        this->DegOrRad = 1;
      }
          
        
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      // Store the pointer to the model
      this->model = _parent;
      this->jointControl = this->model->GetJointController();
      this->joints = this->jointControl->GetJoints();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ROSController_iCub::OnUpdate, this, _1));
      
      string pubTopicName = this->model->GetName() + "/joints";
      this->jointsPublisher = this->nh.advertise<sensor_msgs::JointState>(pubTopicName, 1);
    
      for (std::map<std::string, gazebo::physics::JointPtr>::iterator it = this->joints.begin(); it != this->joints.end(); ++it)
      {
	string posName = this->model->GetName() + "/" + it->second->GetName() + "/pos";
        ros::Subscriber subTemp = nh.subscribe<std_msgs::Float64>(posName, 1, boost::bind(&ROSController_iCub::callbackpos, this, _1, it->first));
	posSubscriber.push_back(subTemp);
	string velName = this->model->GetName() + "/" + it->second->GetName() + "/vel";
	ros::Subscriber subTemp2 = nh.subscribe<std_msgs::Float64>(velName, 1, boost::bind(&ROSController_iCub::callbackvel, this, _1, it->first));
	velSubscriber.push_back(subTemp2);
      }
      //eye version
      string posNameVs = this->model->GetName() + "/" + "eye_version" + "/pos";
      ros::Subscriber subTempVsPos = nh.subscribe<std_msgs::Float64>(posNameVs, 1, boost::bind(&ROSController_iCub::callbackpos, this, _1, this->model->GetName() + "::eye_version"));
      posSubscriber.push_back(subTempVsPos);
      string velNameVs = this->model->GetName() + "/" + "eye_version" + "/vel";
      ros::Subscriber subTempVsVel = nh.subscribe<std_msgs::Float64>(velNameVs, 1, boost::bind(&ROSController_iCub::callbackvel, this, _1, this->model->GetName() + "::eye_version"));
      velSubscriber.push_back(subTempVsVel);
      //eye vergence
      string posNameVg = this->model->GetName() + "/" + "eye_vergence" + "/pos";
      ros::Subscriber subTempVgPos = nh.subscribe<std_msgs::Float64>(posNameVg, 1, boost::bind(&ROSController_iCub::callbackpos, this, _1, this->model->GetName() + "::eye_vergence"));
      posSubscriber.push_back(subTempVgPos);
      string velNameVg = this->model->GetName() + "/" + "eye_vergence" + "/vel";
      ros::Subscriber subTempVgVel = nh.subscribe<std_msgs::Float64>(velNameVg, 1, boost::bind(&ROSController_iCub::callbackvel, this, _1, this->model->GetName() + "::eye_vergence"));
      velSubscriber.push_back(subTempVgVel);
      
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      this->jointControl->Update();
      vector<string> joints_name;
      
      sensor_msgs::JointState msg;
      msg.header.stamp = ros::Time::now();
      for (std::map<std::string, gazebo::physics::JointPtr>::iterator it = this->joints.begin(); it != this->joints.end(); ++it)
      {
	msg.name.push_back(it->second->GetName());
	msg.position.push_back(it->second->GetAngle(0).Radian() * this->DegOrRad);
	msg.velocity.push_back(it->second->GetVelocity(0) * this->DegOrRad);
	msg.effort.push_back(it->second->GetForce(0));
      }
      gazebo::physics::JointPtr left_eye, right_eye;
      left_eye = this->joints.at(this->model->GetName() + "::left_eye_pan");
      right_eye = this->joints.at(this->model->GetName() + "::right_eye_pan");
      //eye version
      msg.name.push_back("eye_version");
      double vs_pos = (left_eye->GetAngle(0).Radian() + right_eye->GetAngle(0).Radian()) / 2.0;
      msg.position.push_back(vs_pos * this->DegOrRad);
      double vs_vel = (left_eye->GetVelocity(0) + right_eye->GetVelocity(0)) / 2.0;
      msg.velocity.push_back(vs_vel * this->DegOrRad);
      msg.effort.push_back(0.0);
      //eye vergence
      msg.name.push_back("eye_vergence");
      double vg_pos = left_eye->GetAngle(0).Radian() - right_eye->GetAngle(0).Radian();
      msg.position.push_back(vg_pos * this->DegOrRad);
      double vg_vel = left_eye->GetVelocity(0) - right_eye->GetVelocity(0);
      msg.velocity.push_back(vg_vel * this->DegOrRad);
      msg.effort.push_back(0.0);
      this->jointsPublisher.publish(msg);
    }

    // Pointer to the model
  private: 
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    gazebo::physics::JointControllerPtr jointControl;
    std::map<std::string, gazebo::physics::JointPtr> joints;
    
    ros::NodeHandle nh;
    ros::Publisher jointsPublisher;
    vector<ros::Subscriber> posSubscriber;
    vector<ros::Subscriber> velSubscriber;
    
    double DegOrRad;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSController_iCub)
} 