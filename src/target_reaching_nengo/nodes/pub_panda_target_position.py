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

class PubPandaTarget:
    def __init__(self):
        self.pred_pos_frame = rospy.get_param('~pred_pos_frame', 'world')
        self.gazebo_pos_frame = rospy.get_param('~gazebo_pos_frame', 'world')

        self.target_points = None
        if rospy.get_param('~is_quadrat_experiment', False):
            scale = rospy.get_param('~scale_kuka_points', 0.8)
            kuka_quadrat_points = [Point(0.25, -0.6, 0.5), Point(0.25, -0.6, 0.9), Point(-0.25, -0.6, 0.9), Point(-0.25, -0.6, 0.5)]
            self.target_points = [Point(p.x * scale, p.y * scale, p.z * scale) for p in kuka_quadrat_points]
        if rospy.get_param('~is_multi_robot_experiment', False):
            self.target_points = [Point(0.25, -0.6, 0.4), Point(0.1, -0.6, 0.8), Point(-0.25, -0.6, 0.4)]

        self.standby_target = to_point_stamped(self.pred_pos_frame, Point(0.25, -0.6, 0.7))
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

        self.set_standby_target_server = rospy.Service('/panda/set_standby_target', Trigger, self.set_standby_target)
        self.remove_standby_target_server = rospy.Service('/panda/remove_standby_target', Trigger, self.remove_standby_target)

    def set_standby_target(self, req):
        self.target_position = self.standby_target
        return {'success': True, 'message': 'set_standby_target'}

    def remove_standby_target(self, req):
        self.target_position = None
        return {'success': True, 'message': 'remove_standby_target'}

    def error_cb(self, msg):
        self.error = msg.data
        has_error = any(self.error)
        if has_error:
            return
        if self.waiting_for_next_position:
            now = rospy.Time.now()
            if (now - self.waiting_start_time) > rospy.Duration(self.next_target_waiting_seconds):
                self.next_target_position()
        else:
            self.waiting_for_next_position = True
            self.waiting_start_time = rospy.Time.now()

    def next_target_position(self):
        self.waiting_for_next_position = False
        self.current_target_position = (self.current_target_position + 1) % len(self.target_points)
        self.target_position.point = self.target_points[self.current_target_position]

    def publish_target_position(self):
        if self.target_position is not None:
            self.target_position_pub.publish(self.target_position)

if __name__== '__main__':
    rospy.init_node('pub_target')
    pub_target = PubPandaTarget()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        pub_target.publish_target_position()
        rate.sleep()
