#!/usr/bin/python
import numpy as np
import rospy
import std_msgs
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sacc_rec_grasp.srv import GraspingState, GraspingStateResponse
import actionlib


class HandManager(object):
    def __init__(self):
        self.pose_server = rospy.Service('grasping_state', GraspingState, self.change_grasping_state)

        # the preset positions => should be loaded from a yaml file
        self.home_pose = [0., 0., 0.0, 0.0, 0.0, 0., 0., 0., 0.]
        pre_grasp_pose_ball = [0.32, 0.2, 0.06, 0.05, 0.0, 0.04, 0.03, 0.2, 0.09]

        pre_grasp_pose_bottle = [0.43, 0.2, 0.06, 0.05, 0.0, 0.04, 0.03, 0.2, 0.99]

        pre_grasp_pose_pen = [0.27, 0.32016, 0.1676829, 0.37352, 0.0399245, 0.0, 0.0, 0.271712, 0.89]

        self.pre_grasp = {'ball': pre_grasp_pose_ball, 'bottle': pre_grasp_pose_bottle, 'pen': pre_grasp_pose_pen}
        grasp_pose_ball = [0.32, 0.78, 0.2, 0.76, 0.14, 0.88, 0.88, 0.49, 0.09]

        grasp_pose_bottle = [0.43, 0.8004000000000001, 0.5030487, 0.74704, 0.5030487, 0.687225, 0.608685, 0.38816000000000006, 0.99]

        grasp_pose_pen = [0.27, 0.51, 0.69, 0.62, 0.57, 0.24, 0.32, 0.45, 0.95]

        self.grasp = {'ball': grasp_pose_ball, 'bottle': grasp_pose_bottle, 'pen': grasp_pose_pen}
        self.drop_pose = [0., 0., 0.0, 0.0, 0.0, 0., 0., 0., 0.]
        self.hand_joint_state_publisher = rospy.Publisher('svh_ros_control_node/svh_pos_based_traj_controller_hand/command', JointTrajectory, queue_size=10)

    def change_grasping_state(self, req):
        if req.pose == "home":
            target_positions = self.home_pose
        elif req.pose == "pre_grasp":
            target_positions = self.pre_grasp[req.label]
        elif req.pose == "grasp":
            target_positions = self.grasp[req.label]
        elif req.pose == "drop":
            target_positions = self.drop_pose
        else:
            rospy.logfatal("Pose reference is invalid: {}".format(req.pose))
            return GraspingStateResponse(False)

        self.publish_pose(target_positions)
        return GraspingStateResponse(True)

    def publish_pose(self, joint_positions):
        joint_state = JointTrajectory()
        joint_state.joint_names = [
            "svh_hand_Finger_Spread",
            "svh_hand_Index_Finger_Distal",
            "svh_hand_Index_Finger_Proximal",
            "svh_hand_Middle_Finger_Distal",
            "svh_hand_Middle_Finger_Proximal",
            "svh_hand_Pinky",
            "svh_hand_Ring_Finger",
            "svh_hand_Thumb_Flexion",
            "svh_hand_Thumb_Opposition"
        ]
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(0.2)
        joint_state.points = [ point ]

        joint_state.header.stamp = rospy.Time.now()
        self.hand_joint_state_publisher.publish(joint_state)

def main():
    rospy.init_node('hand_manager')
    hand_manager = HandManager()
    rospy.spin()


if __name__ == '__main__':
    main()
