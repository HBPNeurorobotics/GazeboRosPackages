#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from geometry_msgs.msg import PointStamped

def get_model_state(model_name, reference_frame):
    model_state = ModelState()
    model_state.model_name = model_name
    model_state.pose.orientation.x = 0.0
    model_state.pose.orientation.y = 0.0
    model_state.pose.orientation.z = 0.0
    model_state.pose.orientation.w = 1.0
    model_state.scale.x = 1.0
    model_state.scale.y = 1.0
    model_state.scale.z = 1.0
    model_state.twist.linear.z = 0.0
    model_state.twist.angular.x = 0.0
    model_state.twist.angular.y = 0.0
    model_state.twist.angular.z = 0.0
    model_state.reference_frame = reference_frame
    return model_state

class PubTargetAsGazeboSphere:
    def __init__(self):
        target_position_topic = rospy.get_param('~target_position_topic', '/target_position')
        self.target_position_pub = rospy.Subscriber(target_position_topic, PointStamped, self.target_position_cb, queue_size=1)
        gazebo_target_model_name = rospy.get_param('~gazebo_target_model_name', 'unit_sphere_1')
        rospy.loginfo("model_name: {}".format(gazebo_target_model_name))
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        gazebo_target_reference_frame = rospy.get_param('~gazebo_target_reference_frame', 'world')
        self.target_model_state = get_model_state(gazebo_target_model_name, gazebo_target_reference_frame)
        self.gazebo_target_base_x = rospy.get_param('~gazebo_target_base_x', 1.0)
        self.gazebo_target_base_y = rospy.get_param('~gazebo_target_base_y', -0.6)
        self.gazebo_target_base_z = rospy.get_param('~gazebo_target_base_z', 0.6)

    def set_pos(self, position):
        self.target_model_state.pose.position.x = position.x + self.gazebo_target_base_x
        self.target_model_state.pose.position.y = position.y + self.gazebo_target_base_y
        self.target_model_state.pose.position.z = position.z + self.gazebo_target_base_z
        self.set_model_state(self.target_model_state)

    def target_position_cb(self, target_point_stamped):
        self.set_pos(target_point_stamped.point)

if __name__== '__main__':
    rospy.init_node('pub_target_as_marker')
    pub_target_as_marker = PubTargetAsGazeboSphere()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()
