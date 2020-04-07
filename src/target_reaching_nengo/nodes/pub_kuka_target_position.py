#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from gazebo_msgs.msg import LinkStates

def to_point_stamped(frame_id, position):
    return PointStamped(header=Header(stamp=rospy.Time.now(),
                        frame_id=frame_id),
                        point=position)

class PubKukaTarget:
    def __init__(self):
        self.target_position = None

        self.target_base_x = rospy.get_param('~target_base_x', 1.4)
        self.target_base_y = rospy.get_param('~target_base_y', -0.6)
        self.target_base_z = rospy.get_param('~target_base_z', 0.6)


        self.pred_pos_frame = rospy.get_param('~pred_pos_frame', 'world')
        self.gazebo_pos_frame = rospy.get_param('~gazebo_pos_frame', 'world')

        #self.gazebo_target_link_name = rospy.get_param('~gazebo_target_link_name', 'ball::ball')
        #self.get_model_state = rospy.Subscriber('/gazebo/link_states', LinkStates, self.gazebo_link_states_cb, queue_size=1)

        pred_pos_topic = rospy.get_param('~pred_pos_topic', '/pred_pos')
        self.pred_pos_sub = rospy.Subscriber(pred_pos_topic, Float32MultiArray, self.pred_pos_cb, queue_size=1)

        self.error = None
        error_topic = rospy.get_param('~error_topic', '/error')
        self.error_sub = rospy.Subscriber(error_topic, Float64MultiArray, self.error_cb, queue_size=1)

        target_position_topic = rospy.get_param('~target_position_topic', '/target_position')
        self.target_position_pub = rospy.Publisher(target_position_topic, PointStamped, queue_size=1)

        self.should_subtract_world_offset = rospy.get_param('should_subtract_world_offset', True)
        if self.should_subtract_world_offset:
            self.world_offset_x = 1.4
            self.world_offset_y = -0.6
            self.world_offset_z = 0.6

    def subtract_world_offset(self, position):
        position[0] -= self.world_offset_x
        position[1] -= self.world_offset_y
        position[2] -= self.world_offset_z
        position[2] += 0.1
        return position

    def gazebo_link_states_cb(self, link_states):
        gazebo_target_position = None
        try:
            gazebo_target_position = link_states.pose[link_states.name.index(self.gazebo_target_link_name)].position
            gazebo_target_position.x -= self.target_base_x
            gazebo_target_position.y -= self.target_base_y
            gazebo_target_position.z -= self.target_base_z
        except ValueError as e:
            rospy.loginfo(str(e))
        if gazebo_target_position is not None:
            self.target_position = to_point_stamped(self.gazebo_pos_frame, gazebo_target_position)

    def error_cb(self, data):
        self.error = data.data

    def pred_pos_cb(self, data):
        predictions = [e for e in data.data if e == e]
        if predictions == []:
            return
        #has_nan = any([e != e for e in data.data])
        #if has_nan:
            #return
        pred_pos = predictions[-3:]
        if self.should_subtract_world_offset:
            pred_pos = self.subtract_world_offset(pred_pos)
        rospy.loginfo("pred_pos: {}".format(pred_pos))
        if pred_pos[0] < -0.5:
            return
        #if len(predictions) >= 10:
            #if pred_pos[0] > 0.5 or (self.error and self.error[0] < 3):
                #index = int(0.7 * len(predictions))
                #pred_pos = predictions[index:index+3]
            #if self.error and self.error[0] < 2:
                #index = int(0.3 * len(predictions))
                #pred_pos = predictions[index:index+3]
        pred_pos_pt = Point(pred_pos[0], pred_pos[1], pred_pos[2])
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
