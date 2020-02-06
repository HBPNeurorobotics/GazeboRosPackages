#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

def get_sphere_marker(scale=0.2, transparency=0.7):
    marker = Marker()
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.a = transparency
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    return marker

class PubTargetAsMarker:
    def __init__(self):
        target_position_topic = rospy.get_param('~target_position_topic', '/target_position')
        self.target_position_pub = rospy.Subscriber(target_position_topic, PointStamped, self.target_position_cb, queue_size=1)

        sphere_marker_pub_topic = rospy.get_param('~sphere_marker_pub_topic', '/sphere_marker')
        self.sphere_marker_pub = rospy.Publisher(sphere_marker_pub_topic, Marker, queue_size=1)
        self.sphere_marker = get_sphere_marker()

    def target_position_cb(self, target_position):
        self.sphere_marker.header = target_position.header
        self.sphere_marker.pose.position = target_position.point
        self.sphere_marker_pub.publish(self.sphere_marker)

if __name__== '__main__':
    rospy.init_node('pub_target_as_marker')
    pub_target_as_marker = PubTargetAsMarker()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()
