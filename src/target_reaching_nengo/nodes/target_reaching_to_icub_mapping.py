#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Trigger

class TargetReachingToICUBMapping:
    def __init__(self):
        neck_joint_cmd_pos_name = rospy.get_param('~neck_joint_cmd_pos_name', '/icub/neck_1/pos')
        l_wrist_1_joint_cmd_pos_name = rospy.get_param('~l_wrist_1_joint_cmd_pos_name', '/icub/l_wrist_1/pos')
        l_wrist_prosup_joint_cmd_pos_name = rospy.get_param('~l_wrist_prosup_joint_cmd_pos_name', '/icub/l_wrist_prosup/pos')
        l_wrist_pitch_joint_cmd_pos_name = rospy.get_param('~l_wrist_pitch_joint_cmd_pos_name', '/icub/l_wrist_pitch/pos')
        l_wrist_pitch_joint_cmd_vel_name = rospy.get_param('~l_wrist_pitch_joint_cmd_vel_name', '/icub/l_wrist_pitch/vel')
        l_wrist_yaw_joint_cmd_pos_name = rospy.get_param('~l_wrist_yaw_joint_cmd_pos_name', '/icub/l_wrist_yaw/pos')
        l_wrist_yaw_joint_cmd_vel_name = rospy.get_param('~l_wrist_yaw_joint_cmd_vel_name', '/icub/l_wrist_yaw/vel')
        l_hand_joint_cmd_pos_name = rospy.get_param('~l_hand_joint_cmd_pos_name', '/icub/l_hand/pos')
        self.neck_joint_cmd_pub = rospy.Publisher(neck_joint_cmd_pos_name, Float64, queue_size=1)
        self.l_wrist_1_joint_cmd_pub = rospy.Publisher(l_wrist_1_joint_cmd_pos_name, Float64, queue_size=1)
        self.l_wrist_prosup_joint_cmd_pub = rospy.Publisher(l_wrist_prosup_joint_cmd_pos_name, Float64, queue_size=1)
        self.l_wrist_pitch_joint_cmd_pub = rospy.Publisher(l_wrist_pitch_joint_cmd_pos_name, Float64, queue_size=1)
        self.l_wrist_pitch_joint_cmd_vel_pub = rospy.Publisher(l_wrist_pitch_joint_cmd_vel_name, Float64, queue_size=1)
        self.l_wrist_yaw_joint_cmd_pub = rospy.Publisher(l_wrist_yaw_joint_cmd_pos_name, Float64, queue_size=1)
        self.l_wrist_yaw_joint_cmd_vel_pub = rospy.Publisher(l_wrist_yaw_joint_cmd_vel_name, Float64, queue_size=1)
        self.l_hand_joint_cmd_pub = rospy.Publisher(l_hand_joint_cmd_pos_name, Float64, queue_size=1)

        self.arm_joint_cmds = {}

        arm_1_joint_cmd_pos_name = rospy.get_param('~arm_1_joint_cmd_pos_name', '/icub/joint_1')
        arm_2_joint_cmd_pos_name = rospy.get_param('~arm_2_joint_cmd_pos_name', '/icub/joint_2')
        arm_3_joint_cmd_pos_name = rospy.get_param('~arm_3_joint_cmd_pos_name', '/icub/joint_3')

        self.arm_1_joint_cmd_sub = rospy.Subscriber(arm_1_joint_cmd_pos_name, Float64, self.cmd_callback, callback_args="joint_1", queue_size=1)
        self.arm_2_joint_cmd_sub = rospy.Subscriber(arm_2_joint_cmd_pos_name, Float64, self.cmd_callback, callback_args="joint_2", queue_size=1)
        self.arm_3_joint_cmd_sub = rospy.Subscriber(arm_3_joint_cmd_pos_name, Float64, self.cmd_callback, callback_args="joint_3", queue_size=1)

        joint_1 = rospy.get_param('~joint_1', '/icub/torso_yaw/pos')
        joint_2 = rospy.get_param('~joint_2', '/icub/torso_pitch/pos')
        joint_3 = rospy.get_param('~joint_3', '/icub/l_elbow/pos')

        self.joint_1_pub = rospy.Publisher(joint_1, Float64, queue_size=1)
        self.joint_2_pub = rospy.Publisher(joint_2, Float64, queue_size=1)
        self.joint_3_pub = rospy.Publisher(joint_3, Float64, queue_size=1)

    def cmd_callback(self, cmd, joint_name):
        self.arm_joint_cmds[joint_name] = cmd.data

    def publish_joint_cmds(self):
        if "joint_1" in self.arm_joint_cmds:
            self.joint_1_pub.publish(self.arm_joint_cmds["joint_1"])
        if "joint_2" in self.arm_joint_cmds:
            self.joint_2_pub.publish(self.arm_joint_cmds["joint_2"])
        if "joint_3" in self.arm_joint_cmds:
            self.joint_3_pub.publish(self.arm_joint_cmds["joint_3"])

    def publish_zero_to_unnecessary_joints(self):
        #self.neck_joint_cmd_pub.publish(0.0)
        #self.l_wrist_1_joint_cmd_pub.publish(0.0)
        #self.l_wrist_prosup_joint_cmd_pub.publish(0.0)
        self.l_wrist_pitch_joint_cmd_pub.publish(0.0)
        #self.l_wrist_pitch_joint_cmd_vel_pub.publish(0.0)
        self.l_wrist_yaw_joint_cmd_pub.publish(0.0)
        #self.l_wrist_yaw_joint_cmd_vel_pub.publish(0.0)
        #self.l_hand_joint_cmd_pub.publish(0.0)

def main(argv=None):
    rospy.init_node("TargetReachingToICUBMapping")
    mapping = TargetReachingToICUBMapping()
    rospy.loginfo("TargetReachingToICUBMapping initialized")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        mapping.publish_joint_cmds()
        rate.sleep()

if __name__ == "__main__":
    main()
