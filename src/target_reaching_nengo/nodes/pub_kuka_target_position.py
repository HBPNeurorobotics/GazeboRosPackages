#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Float64, Header
from geometry_msgs.msg import PointStamped, Point
from gazebo_msgs.msg import LinkStates
from std_srvs.srv import Trigger

def to_point_stamped(frame_id, position):
    return PointStamped(header=Header(stamp=rospy.Time.now(),
                        frame_id=frame_id),
                        point=position)

class PubKukaTarget:
    def __init__(self):
        self.pred_pos_frame = rospy.get_param('~pred_pos_frame', 'world')
        self.gazebo_pos_frame = rospy.get_param('~gazebo_pos_frame', 'world')

        self.target_points = None
        if rospy.get_param('~is_quadrat_experiment', False):
            self.target_points = [Point(0.25, -0.6, 0.5), Point(0.25, -0.6, 0.9), Point(-0.25, -0.6, 0.9), Point(-0.25, -0.6, 0.5)]
        if rospy.get_param('~is_multi_robot_experiment', False):
            self.target_points = [Point(0.25, -0.6, 0.5), Point(0.1, -0.6, 0.9), Point(-0.25, -0.6, 0.5)]
        if rospy.get_param('~is_error_experiment', False):
            self.target_points = [Point(0.25, -0.6, 0.7)]

        self.standby_target = to_point_stamped(self.pred_pos_frame, Point(0.1, -0.35, 0.7))
        self.error = None
        error_topic = rospy.get_param('~error_topic', '/error')
        self.error_sub = rospy.Subscriber(error_topic, Float64MultiArray, self.error_cb, queue_size=1)

        self.target_position = None
        target_position_topic = rospy.get_param('~target_position_topic', '/target_position')
        self.target_position_pub = rospy.Publisher(target_position_topic, PointStamped, queue_size=1)

        self.using_target_points = self.target_points != None
        if self.using_target_points:
            self.current_target_position = 0
            self.waiting_for_next_position = False
            self.waiting_start_time = None
            self.next_target_waiting_seconds = rospy.get_param('~next_target_waiting_seconds', 3.0)
            self.target_position = to_point_stamped(self.pred_pos_frame, self.target_points[self.current_target_position])
        else:
            self.target_position = self.standby_target
            self.is_standby_target = True
            pred_pos_topic = rospy.get_param('~pred_pos_topic', '/pred_pos')
            self.pred_pos_sub = rospy.Subscriber(pred_pos_topic, Float32MultiArray, self.pred_pos_cb, queue_size=1)
            self.should_subtract_world_offset = rospy.get_param('~should_subtract_world_offset', True)
            if self.should_subtract_world_offset:
                self.world_offset_x = rospy.get_param('~gazebo_target_base_x', 1.0)
                self.world_offset_y = rospy.get_param('~gazebo_target_base_y', -0.6)
                self.world_offset_z = rospy.get_param('~gazebo_target_base_z', 0.6)

        gripper_left_topic = rospy.get_param('~gripper_left_topic', '/iiwa/gripper_left_effort_controller/command')
        gripper_right_topic = rospy.get_param('~gripper_right_topic', '/iiwa/gripper_right_effort_controller/command')
        self.gripper_left_pub = rospy.Publisher(gripper_left_topic, Float64, queue_size=1)
        self.gripper_right_pub = rospy.Publisher(gripper_right_topic, Float64, queue_size=1)

        self.set_standby_target_server = rospy.Service('/iiwa/set_standby_target', Trigger, self.set_standby_target)
        self.remove_standby_target_server = rospy.Service('/iiwa/remove_standby_target', Trigger, self.remove_standby_target)

    def set_standby_target(self, req):
        self.is_standby_target = True
        self.target_position = self.standby_target
        return {'success': True, 'message': 'set_standby_target'}

    def remove_standby_target(self, req):
        self.is_standby_target = False
        self.target_position = None
        return {'success': True, 'message': 'remove_standby_target'}

    def open_gripper(self, cmd=-10.0):
        self.gripper_left_pub.publish(cmd)
        self.gripper_right_pub.publish(cmd)

    def close_gripper(self, cmd=10.0):
        self.gripper_left_pub.publish(cmd)
        self.gripper_right_pub.publish(cmd)

    def subtract_world_offset(self, position):
        position[0] -= self.world_offset_x
        position[1] -= self.world_offset_y
        position[1] += 0.03
        position[2] -= self.world_offset_z
        position[2] += 0.08
        return position

    def error_cb(self, msg):
        self.error = msg.data
        has_error = any(self.error)
        if has_error:
            self.open_gripper()
            return
        if self.using_target_points:
            if self.waiting_for_next_position:
                now = rospy.Time.now()
                if (now - self.waiting_start_time) > rospy.Duration(self.next_target_waiting_seconds):
                    self.next_target_position()
            else:
                self.waiting_for_next_position = True
                self.waiting_start_time = rospy.Time.now()
        else:
            if not self.is_standby_target:
                self.close_gripper()

    def next_target_position(self):
        self.waiting_for_next_position = False
        self.current_target_position = (self.current_target_position + 1) % len(self.target_points)
        self.target_position.point = self.target_points[self.current_target_position]

    def pred_pos_cb(self, msg):
        predictions = [e for e in msg.data if e == e]
        if predictions == [] or len(predictions) < 3:
            return
        pred_pos = predictions[-3:]

        if self.target_position is not None:
            return

        if self.should_subtract_world_offset:
            pred_pos = self.subtract_world_offset(pred_pos)
        rospy.loginfo("pred_pos: {}".format(pred_pos))
        if pred_pos[0] < -0.71:
            return
        pred_pos_pt = Point(-0.1, pred_pos[1], pred_pos[2])
        rospy.loginfo("pred_pos_pt: {}".format(pred_pos_pt))
        self.target_position = to_point_stamped(self.pred_pos_frame, pred_pos_pt)

    def publish_target_position(self):
        if self.target_position is not None:
            self.target_position_pub.publish(self.target_position)

if __name__== '__main__':
    rospy.init_node('pub_target')
    pub_target = PubKukaTarget()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        pub_target.publish_target_position()
        rate.sleep()
