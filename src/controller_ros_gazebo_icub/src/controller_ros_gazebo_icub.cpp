#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <map>
#include <string>
#include <vector>
#include <mutex>

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
      // get mutex to implement thread-safe reset of controller and avoid async, contradictory pos/vel controller setting
      std::lock_guard<std::mutex> lock(this->mutex);

      // control method
      string jointNameCtrl1 = jointId;
      string jointNameCtrl2 = jointId;
      if ((jointId == this->model->GetName() + "::eye_version") ||
          (jointId == this->model->GetName() + "::eye_vergence"))
      {
        jointNameCtrl1 = this->model->GetName() + "::left_eye_pan";
        jointNameCtrl2 = this->model->GetName() + "::right_eye_pan";
      }
      std::map<std::string, double> positions = this->jointControl->GetPositions();
      std::map<std::string, double> velocities = this->jointControl->GetVelocities();
      std::map<std::string, double>::iterator findIt = velocities.find(jointNameCtrl1);

      if (findIt != velocities.end())
      {
        cout << "CHANGED CONTROL POLICY JOINT " << jointId << endl;

        this->jointControl->Reset();

        cout << "POSITIONS:" << endl;
        for (std::map<std::string, double>::iterator it = positions.begin(); it != positions.end(); ++it)
        {
          if ((it->first != jointNameCtrl1) && (it->first != jointNameCtrl2))
          {
            this->jointControl->SetPositionTarget(it->first, positions[it->first]);
            cout << "OLD ";
          }
          cout << it->first << " " << positions[it->first] << endl;
        }
        cout << "VELOCITIES:" << endl;
        for (std::map<std::string, double>::iterator it = velocities.begin(); it != velocities.end(); ++it)
        {
          if ((it->first != jointNameCtrl1) && (it->first != jointNameCtrl2))
          {
            this->jointControl->SetVelocityTarget(it->first, positions[it->first]);
            cout << "OLD ";
          }
          cout << it->first << " " << positions[it->first] << endl;
        }
      }

      double command = msg->data / this->DegOrRad;
      if (jointId == this->model->GetName() + "::eye_version")
      {
          gazebo::physics::JointPtr left_eye, right_eye;
          left_eye = this->joints.at(this->model->GetName() + "::left_eye_pan");
          right_eye = this->joints.at(this->model->GetName() + "::right_eye_pan");
          double vg = left_eye->Position(0) - right_eye->Position(0);
          double left_pan_pos = command + (vg / 2.0);
          double right_pan_pos = command - (vg / 2.0);
          this->jointControl->SetPositionTarget(this->model->GetName() + "::left_eye_pan", left_pan_pos);
          this->jointControl->SetPositionTarget(this->model->GetName() + "::right_eye_pan", right_pan_pos);
      }
      else if (jointId == this->model->GetName() + "::eye_vergence")
      {
          gazebo::physics::JointPtr left_eye, right_eye;
          left_eye = this->joints.at(this->model->GetName() + "::left_eye_pan");
          right_eye = this->joints.at(this->model->GetName() + "::right_eye_pan");
          double vs = (left_eye->Position(0) + right_eye->Position(0)) / 2.0;
          double left_pan_pos = vs + (command / 2.0);
          double right_pan_pos = vs - (command / 2.0);
          this->jointControl->SetPositionTarget(this->model->GetName() + "::left_eye_pan", left_pan_pos);
          this->jointControl->SetPositionTarget(this->model->GetName() + "::right_eye_pan", right_pan_pos);
      }
      else
      {
        this->jointControl->SetPositionTarget(jointId, command);
      }
    }

    void callbackvel(const std_msgs::Float64::ConstPtr &msg, const std::string &jointId) {
      // get mutex to implement thread-safe reset of controller and avoid async, contradictory pos/vel controller setting
      std::lock_guard<std::mutex> lock(this->mutex);

      // control method
      string jointNameCtrl1 = jointId;
      string jointNameCtrl2 = jointId;
      if ((jointId == this->model->GetName() + "::eye_version") ||
          (jointId == this->model->GetName() + "::eye_vergence"))
      {
        jointNameCtrl1 = this->model->GetName() + "::left_eye_pan";
        jointNameCtrl2 = this->model->GetName() + "::right_eye_pan";
      }
      std::map<std::string, double> positions = this->jointControl->GetPositions();
      std::map<std::string, double> velocities = this->jointControl->GetVelocities();
      std::map<std::string, double>::iterator findIt = positions.find(jointNameCtrl1);

      if (findIt != positions.end())
      {
        cout << "CHANGED CONTROL POLICY JOINT " << jointId << endl;

        this->jointControl->Reset();

        cout << "POSITIONS:" << endl;
        for (std::map<std::string, double>::iterator it = positions.begin(); it != positions.end(); ++it)
        {
          if ((it->first != jointNameCtrl1) && (it->first != jointNameCtrl2))
          {
            this->jointControl->SetPositionTarget(it->first, positions[it->first]);
            cout << "OLD ";
          }
          cout << it->first << " " << positions[it->first] << endl;
        }
        cout << "VELOCITIES:" << endl;
        for (std::map<std::string, double>::iterator it = velocities.begin(); it != velocities.end(); ++it)
        {
          if ((it->first != jointNameCtrl1) && (it->first != jointNameCtrl2))
          {
            this->jointControl->SetVelocityTarget(it->first, positions[it->first]);
            cout << "OLD ";
          }
          cout << it->first << " " << positions[it->first] << endl;
        }
      }

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
      {
        this->jointControl->SetVelocityTarget(jointId, command);
      }
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      if (!_sdf->HasElement("degree"))
      {
        std::cout << "Missing parameter <degree> in ROSController_iCub, default to radiants" << std::endl;
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
        std::cout << "Wrong value for the parameter <degree> in ROSController_iCub, default to radiants" << std::endl;
        this->DegOrRad = 1;
      }
    }

    void Init()
    {
      cout << "[ROS CONTROLLER] controller initialized" << endl;

      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      this->jointControl = this->model->GetJointController();
      this->joints = this->jointControl->GetJoints();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ROSController_iCub::OnUpdate, this, _1));

      string pubTopicName = this->model->GetName() + "/joints";
      this->jointsPublisher = this->nh.advertise<sensor_msgs::JointState>(pubTopicName, 1);

      cout << "[ROS CONTROLLER] publisher initialized" << endl;

      for (std::map<std::string, gazebo::physics::JointPtr>::iterator it = this->joints.begin(); it != this->joints.end(); ++it)
      {
        string posName = this->model->GetName() + "/" + it->second->GetName() + "/pos";
        ros::Subscriber subTemp = nh.subscribe<std_msgs::Float64>(posName, 1, boost::bind(&ROSController_iCub::callbackpos, this, _1, it->first));
        posSubscriber.push_back(subTemp);
        this->model->SaveControllerActuatorRosTopics(posName, "std_msgs/Float64");

        string velName = this->model->GetName() + "/" + it->second->GetName() + "/vel";
        ros::Subscriber subTemp2 = nh.subscribe<std_msgs::Float64>(velName, 1, boost::bind(&ROSController_iCub::callbackvel, this, _1, it->first));
        velSubscriber.push_back(subTemp2);
        this->model->SaveControllerActuatorRosTopics(velName, "std_msgs/Float64");
      }
      //eye version
      string posNameVs = this->model->GetName() + "/" + "eye_version" + "/pos";
      ros::Subscriber subTempVsPos = nh.subscribe<std_msgs::Float64>(posNameVs, 1, boost::bind(&ROSController_iCub::callbackpos, this, _1, this->model->GetName() + "::eye_version"));
      posSubscriber.push_back(subTempVsPos);
      this->model->SaveControllerActuatorRosTopics(posNameVs, "std_msgs/Float64");

      string velNameVs = this->model->GetName() + "/" + "eye_version" + "/vel";
      ros::Subscriber subTempVsVel = nh.subscribe<std_msgs::Float64>(velNameVs, 1, boost::bind(&ROSController_iCub::callbackvel, this, _1, this->model->GetName() + "::eye_version"));
      velSubscriber.push_back(subTempVsVel);
      this->model->SaveControllerActuatorRosTopics(velNameVs, "std_msgs/Float64");
      //eye vergence
      string posNameVg = this->model->GetName() + "/" + "eye_vergence" + "/pos";
      ros::Subscriber subTempVgPos = nh.subscribe<std_msgs::Float64>(posNameVg, 1, boost::bind(&ROSController_iCub::callbackpos, this, _1, this->model->GetName() + "::eye_vergence"));
      posSubscriber.push_back(subTempVgPos);
      this->model->SaveControllerActuatorRosTopics(posNameVg, "std_msgs/Float64");

      string velNameVg = this->model->GetName() + "/" + "eye_vergence" + "/vel";
      ros::Subscriber subTempVgVel = nh.subscribe<std_msgs::Float64>(velNameVg, 1, boost::bind(&ROSController_iCub::callbackvel, this, _1, this->model->GetName() + "::eye_vergence"));
      velSubscriber.push_back(subTempVgVel);
      this->model->SaveControllerActuatorRosTopics(velNameVg, "std_msgs/Float64");

      cout << "[ROS CONTROLLER] subscribers initialized" << endl;

    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      this->jointControl->Update();
      vector<string> joints_name;

      gazebo::common::Time gazeboSimTime = this->model->GetWorld()->SimTime();
      sensor_msgs::JointState msg;
      /* TODO: msg.header.seq should be populated too, ros::Time::now() automatically creates also a sequence number for the message
         whereas this->model->GetWorld()->SimTime() does not. This is not a problem currently as we are never using that value, but
         for the sake of completeness a way to add this value properly should be found. */
      msg.header.stamp.sec = gazeboSimTime.sec;
      msg.header.stamp.nsec = gazeboSimTime.nsec;
      for (std::map<std::string, gazebo::physics::JointPtr>::iterator it = this->joints.begin(); it != this->joints.end(); ++it)
      {
        msg.name.push_back(it->second->GetName());
        msg.position.push_back(it->second->Position(0) * this->DegOrRad);
        msg.velocity.push_back(it->second->GetVelocity(0) * this->DegOrRad);
        msg.effort.push_back(it->second->GetForce(0));
      }
      gazebo::physics::JointPtr left_eye, right_eye;
      left_eye = this->joints.at(this->model->GetName() + "::left_eye_pan");
      right_eye = this->joints.at(this->model->GetName() + "::right_eye_pan");
      //eye version
      msg.name.push_back("eye_version");
      double vs_pos = (left_eye->Position() + right_eye->Position(0)) / 2.0;
      msg.position.push_back(vs_pos * this->DegOrRad);
      double vs_vel = (left_eye->GetVelocity(0) + right_eye->GetVelocity(0)) / 2.0;
      msg.velocity.push_back(vs_vel * this->DegOrRad);
      msg.effort.push_back(0.0);
      //eye vergence
      msg.name.push_back("eye_vergence");
      double vg_pos = left_eye->Position() - right_eye->Position(0);
      msg.position.push_back(vg_pos * this->DegOrRad);
      double vg_vel = left_eye->GetVelocity(0) - right_eye->GetVelocity(0);
      msg.velocity.push_back(vg_vel * this->DegOrRad);
      msg.effort.push_back(0.0);
      this->jointsPublisher.publish(msg);
    }

    // Pointer to the model
  private:
    std::mutex mutex;
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
