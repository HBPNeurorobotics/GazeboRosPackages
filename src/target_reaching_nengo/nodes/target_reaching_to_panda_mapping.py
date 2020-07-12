#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Trigger

class TargetReachingToKUKAMapping:
    def __init__(self):
        self.has_joint_state = False
        self.has_joint_cmd = False
        self.joint_names = rospy.get_param('~joint_names')
        self.joint_states_topic = rospy.get_param('~joint_states_topic')
        self.joint_state_sub = rospy.Subscriber(self.joint_states_topic, JointState, self.joint_states_callback, queue_size=1)
        self.arm_joint_cmds = {}
        self.last_positions_to_send = []
        self.last_joint_state = [0.0 for i in range(len(self.joint_names))]
        arm_1_joint_cmd_pos_name = rospy.get_param('~arm_1_joint_cmd_pos_name')
        arm_2_joint_cmd_pos_name = rospy.get_param('~arm_2_joint_cmd_pos_name')
        arm_3_joint_cmd_pos_name = rospy.get_param('~arm_3_joint_cmd_pos_name')
        self.arm_1_joint_cmd_sub = rospy.Subscriber(arm_1_joint_cmd_pos_name, Float64, self.cmd_callback, callback_args="arm_1_joint", queue_size=1)
        self.arm_2_joint_cmd_sub = rospy.Subscriber(arm_2_joint_cmd_pos_name, Float64, self.cmd_callback, callback_args="arm_2_joint", queue_size=1)
        self.arm_3_joint_cmd_sub = rospy.Subscriber(arm_3_joint_cmd_pos_name, Float64, self.cmd_callback, callback_args="arm_3_joint", queue_size=1)
        self.received_joint_cmds_pub = rospy.Publisher('/received_joint_cmds', String, queue_size=1)
        follow_joint_trajectory_param = rospy.get_param('~follow_joint_trajectory_param')
        self.arm_traj_client = actionlib.SimpleActionClient(follow_joint_trajectory_param, FollowJointTrajectoryAction)
        self.pos_diff_tolerance = rospy.get_param('~pos_diff_tolerance', 0.009)
        self.arm_3_joint_index = rospy.get_param('~arm_3_joint_index', 3)
        self.move_to_standby_server = rospy.Service('/move_to_standby', Trigger, self._move_to_standby)

    def _move_to_standby(self, req):
        self.move_to_standby()
        self.arm_traj_client.wait_for_result()
        return {'success': True, 'message': 'Moved to standby'}

    def move_to_standby(self, duration=0.5):
        self.arm_traj_client.cancel_all_goals()
        positions_to_send = [0.0 for i in range(len(self.joint_names))]
        arm_goal = FollowJointTrajectoryGoal()
        arm_goal.trajectory.joint_names = self.joint_names
        waypoint = JointTrajectoryPoint()
        for i in range(len(self.joint_names)):
            waypoint.positions = positions_to_send
        waypoint.time_from_start = rospy.Duration.from_sec(duration)
        arm_goal.trajectory.points.append(waypoint)
        self.arm_traj_client.send_goal(arm_goal)

    def cmd_callback(self, cmd, joint_name):
        self.has_joint_cmd = True
        self.arm_joint_cmds[joint_name] = cmd.data

    def joint_states_callback(self, joint_state):
        for i in range(len(self.joint_names)):
            for j in range(len(joint_state.name)):
                if self.joint_names[i] == joint_state.name[j]:
                    self.last_joint_state[i] = joint_state.position[j]
                    self.has_joint_state = True

    def send_arm_trajectory(self, duration=0.5):
        if not self.has_joint_state or not self.has_joint_cmd:
            return
        positions_to_send = self.last_joint_state
        if "arm_1_joint" in self.arm_joint_cmds:
            positions_to_send[0] = self.arm_joint_cmds["arm_1_joint"]
        if "arm_2_joint" in self.arm_joint_cmds:
            positions_to_send[1] = self.arm_joint_cmds["arm_2_joint"]
        if "arm_3_joint" in self.arm_joint_cmds:
            positions_to_send[self.arm_3_joint_index - 1] = self.arm_joint_cmds["arm_3_joint"]
        #TODO: remove this hack for moving 7th joint
        positions_to_send[len(positions_to_send)-2] = 1.7
        positions_to_send[len(positions_to_send)-1] = 1.
        #for i in range(3,6):
            #positions_to_send[i] = 0.0
        #if self.last_positions_to_send:
            #pos_diff = list(map(lambda x,y:abs(x-y), self.last_positions_to_send, positions_to_send))
            #if all(diff <= self.pos_diff_tolerance for diff in pos_diff):
                #return
        #self.last_positions_to_send = positions_to_send
        to_pub = "self.arm_joint_cmds: {}, pos to send: {}".format(self.arm_joint_cmds, positions_to_send)
        self.received_joint_cmds_pub.publish(to_pub)
        self.arm_joint_cmds = {}
        arm_goal = FollowJointTrajectoryGoal()
        arm_goal.trajectory.joint_names = self.joint_names
        waypoint = JointTrajectoryPoint()
        for i in range(len(self.joint_names)):
            waypoint.positions = positions_to_send
        waypoint.time_from_start = rospy.Duration.from_sec(duration)
        arm_goal.trajectory.points.append(waypoint)
        self.arm_traj_client.cancel_all_goals()
        self.arm_traj_client.send_goal(arm_goal)

def main(argv=None):
    rospy.init_node("TargetReachingToKUKAMapping")
    mapping = TargetReachingToKUKAMapping()
    rospy.loginfo("TargetReachingToKUKAMapping initialized")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        mapping.send_arm_trajectory()
        rate.sleep()

if __name__ == "__main__":
    main()
