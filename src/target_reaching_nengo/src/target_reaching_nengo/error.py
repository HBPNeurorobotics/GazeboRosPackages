#!/usr/bin/env python

from __future__ import division

import copy
import math
import os
import sys
from random import uniform

import nengo
import numpy as np

from target_reaching_nengo import Item
from std_msgs.msg import String, Float64MultiArray
from target_reaching_common import Generate_curve
import rospy

import tf
from geometry_msgs.msg import PointStamped

class Error(object):
    def __init__(self, subject_name, threshold, learning = False, n_points = 41, amplitude = 0.2, period = 2 * np.pi, phase_shift = 0, vertical_shift = 0, do_print = False, mult_with_radius = False):
        self.subject        = Item('subject')
        self.tcp            = Item('tcp')
        self.shoulder       = Item('shoulder')
        self.threshold      = threshold

        self.learning       = learning
        self.dif = self.error = [0.0, 0.0, 0.0]

        #cartesian learning
        self.tcp_position_z = 0.0
        self.tcp_position_y = 0.0
        self.value_curve = 0.0
        self.curve_index_normed = 0.0
        self.indice_curve = 0.0
        generate_curve_data_1 = Generate_curve(n_points, amplitude, period, phase_shift, vertical_shift, do_print)
        self.curve_data  =  generate_curve_data_1.data

        error_class_data_topic = rospy.get_param('~error_class_data_topic', '/error_class_data_pub')
        self.error_class_data_pub = rospy.Publisher(error_class_data_topic, String, queue_size=1)
        error_topic = rospy.get_param('~error_topic', '/error')
        self.error_pub = rospy.Publisher(error_topic, Float64MultiArray, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.shoulder_frame = rospy.get_param('~shoulder_frame', 'arm_base_link')
        self.tcp_frame = rospy.get_param('~tcp_frame', 'arm_tcp_link')
        self.tcp.position.x = 0.0
        self.tcp.position.y = 0.0
        self.tcp.position.z = 0.0
        target_position_topic = rospy.get_param('~target_position_topic', '/target_position')
        self.target_position_sub = rospy.Subscriber(target_position_topic, PointStamped, self.target_position_callback, queue_size=1)
        self.last_target_position_received_time = None
        self.target_position_secs_tolerance = rospy.get_param('~target_position_secs_tolerance', 0.5)
        self.map_phi_to_2pi = rospy.get_param('~map_phi_to_2pi', True)

    def target_position_callback(self, data):
        self.subject.position = data.point
        self.subject_frame = data.header.frame_id
        self.last_target_position_received_time = rospy.get_rostime()

        self.subject.vector_to_shoulder = self.transform_position(self.subject.position, self.subject_frame, target_frame = self.shoulder_frame)
        self.tcp.vector_to_shoulder = self.transform_position(self.tcp.position, self.tcp_frame, target_frame = self.shoulder_frame)

        self.subject.polar_pos  =  self.calc_polar(self.subject.vector_to_shoulder.x, self.subject.vector_to_shoulder.y, self.subject.vector_to_shoulder.z)
        self.tcp.polar_pos      =   self.calc_polar(self.tcp.vector_to_shoulder.x, self.tcp.vector_to_shoulder.y, self.tcp.vector_to_shoulder.z)
        self.dif = self.calc_dif()
        if self.learning: self.error = self.calc_error_2()
        else: self.error = self.calc_error()
        error_to_pub = Float64MultiArray()
        error_to_pub.data = self.error
        self.error_pub.publish(error_to_pub)

        subject = "({:.2f}, {:.2f}, {:.2f})".format(self.subject.vector_to_shoulder.x, self.subject.vector_to_shoulder.y, self.subject.vector_to_shoulder.z)
        tcp = "({:.2f}, {:.2f}, {:.2f})".format(self.tcp.vector_to_shoulder.x, self.tcp.vector_to_shoulder.y, self.tcp.vector_to_shoulder.z)
        subject_polar = "({:.2f}, {:.2f}, {:.2f})".format(self.subject.polar_pos[0], self.subject.polar_pos[1], self.subject.polar_pos[2])
        tcp_polar = "({:.2f}, {:.2f}, {:.2f})".format(self.tcp.polar_pos[0], self.tcp.polar_pos[1], self.tcp.polar_pos[2])
        diff = "({:.2f}, {:.2f}, {:.2f})".format(self.dif[0], self.dif[1], self.dif[2])
        to_pub = "[subject, tcp, subject polar, tcp polar, diff]: [" + subject + ", " + tcp + ", " + subject_polar + ", " + tcp_polar + ", " + diff + "]"
        self.error_class_data_pub.publish(to_pub)

    def transform_position(self, position, source_frame, target_frame, transform_waiting_duration=4.0):
        point_stamped = PointStamped()
        point_stamped.header.frame_id = source_frame
        point_stamped.point = position
        self.tf_listener.waitForTransform(source_frame, target_frame, rospy.Time(0),rospy.Duration(transform_waiting_duration))
        point_transformed = self.tf_listener.transformPoint(target_frame, point_stamped)
        return point_transformed.point

    def check_error(self):
        if self.last_target_position_received_time is None:
            return
        now = rospy.get_rostime()
        from_last_target_position_until_now = now - self.last_target_position_received_time
        if from_last_target_position_until_now.to_sec() > self.target_position_secs_tolerance:
            self.error = [0.0, 0.0, 0.0]

    #  um aus main klassen error zu bekommen
    def get_val(self, t):
        self.check_error()
        return self.error

    #  um aus main klassen die position  von tcp zu bekommen
    def get_tcp(self, t):
        pos = self.tcp.position
        if pos is not None: return [pos.x, pos.y, pos.z]
        else: return [0.0, 0.0, 0.0]

    #  um aus main klassen die position von subject/ shoulder zu bekommen
    def get_subject(self, t):
        pos = self.subject.position
        if pos is not None: return [pos.x, pos.y, pos.z]
        else: return [0.0, 0.0, 0.0]

    #  um aus main klassen die positionen von shoulder zu bekommen
    def get_shoulder(self, t):
        pos = self.shoulder.position
        res = None
        if self.shoulder.position is not None and self.tcp.position is not None:
            res = [self.shoulder.position.x - self.tcp.position.x, self.shoulder.position.y - self.tcp.position.y, self.shoulder.position.z - self.tcp.position.z]
        if pos is not None: return [pos.x, pos.y, pos.z]
        else: return [0.0, 0.0, 0.0]





    # berechnet koordinaten im bezug auf shoulder
    def calc_vector(self, p):
        res = copy.copy(p)
        res.x = p.x -self.shoulder.position.x
        res.y = p.y -self.shoulder.position.y
        res.z = p.z -self.shoulder.position.z
        return res

    def check_limit(self, subject):
        if subject.y > 0.0 : subject.y = -0.0001
        return subject

    def calc_polar(self, x, y, z):
        r       = math.sqrt( math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2))
        theta   = math.acos(z / r)
        phi     = math.atan2(y/r, x/r)
        if self.map_phi_to_2pi and phi < 0.:
            phi += 2*np.pi
        return [r, theta, phi]

    def calc_dif(self):
        dif = [self.subject.polar_pos[0] - self.tcp.polar_pos[0],  # r
               self.subject.polar_pos[1] - self.tcp.polar_pos[1],  #theta
               self.subject.polar_pos[2] - self.tcp.polar_pos[2]]  #phi
        return dif

    def calc_error(self):
        error = [0,0,0]
        for i in range(3):
            if abs(self.dif[i]) > self.threshold[i][1]:
                error[i] = 1
                if abs(self.dif[i]) > self.threshold[i][1]    : error[i] = 1
                if abs(self.dif[i]) > self.threshold[i][1] * 5: error[i] = 2
                if abs(self.dif[i]) > self.threshold[i][1] * 10: error[i] = 3
                error[i] = error[i] * (self.dif[i] / abs(self.dif[i]))

            #if i == 0 and error[i] > 0:
                #error[i] = error[i] * 3
        return error

    def calc_error_2(self):

        error = [0,0,0]
        for i in range(3):
            if i is 0:
                if abs(self.dif[i]) > self.threshold[i][1]:
                    error[i] = 5
                    error[i] = error[i] * (self.dif[i] / abs(self.dif[i]))
        return error




   ########################################### L E A R N I N G  - C A R T E S I A N #################################


    def get_tcp_position(self, t):
        return [self.tcp_position_y, self.tcp_position_z]

    def get_value_curve(self, x):
        return [self.indice_curve ,self.value_curve]


    def set_tcp_position(self, x):
        def percentage(part, whole):
            return 100 * float(part)/float(whole)

        dist = [0.0, 0.0]
        punkt_1 = copy.copy(self.shoulder.position) # shoulder position
        punkt_2 = copy.copy(self.subject.position)  # subject position
        punkt_3 = copy.copy(self.tcp.position)      # tcp position
        if punkt_1 is not None and punkt_2 is not None and punkt_3 is not None:
            initial_tcp_y  = punkt_1.y +0.3
            initial_tcp_z  = punkt_1.z
            subject_y   = punkt_2.y
            subject_z   = punkt_2.z
            tcp_y       = punkt_3.y
            tcp_z       = punkt_3.z

            if initial_tcp_y < 0 or initial_tcp_z < 0 or subject_y < 0 or subject_z < 0 or tcp_y < 0 or tcp_z < 0:
                print 'WARNING: Calculation of error does not work with negative values.'
            else:

                if abs(subject_z - initial_tcp_z) > 0.01: print 'shoulder and subject schould be in one line (same z value)'
                avg_z = (initial_tcp_z + subject_z)/2
                self.tcp_position_z = tcp_z - avg_z
                length_y =  subject_y - initial_tcp_y
                pos_y = tcp_y - initial_tcp_y
                self.tcp_position_y = max(0.0, min(1.0, percentage(pos_y, length_y) /100))



    def set_value_curve(self, x):
        curve_index_normed = 0.0
        dist = [0.0, 0.0]
        punkt_1 = copy.copy(self.shoulder.position) # shoulder position
        punkt_2 = copy.copy(self.subject.position)  # subject position
        punkt_3 = copy.copy(self.tcp.position)      # tcp position
        if punkt_1 is not None and punkt_2 is not None and punkt_3 is not None:
            initial_tcp_y  = punkt_1.y +0.3
            initial_tcp_z  = punkt_1.z
            subject_y   = punkt_2.y
            subject_z   = punkt_2.z
            tcp_y       = punkt_3.y
            tcp_z       = punkt_3.z

            if initial_tcp_y < 0 or initial_tcp_z < 0 or subject_y < 0 or subject_z < 0 or tcp_y < 0 or tcp_z < 0:
                print 'WARNING: Calculation of error does not work with negative values.'
            else:

                if abs(subject_z - initial_tcp_z) > 0.01: print 'shoulder and subject schould be in one line (same z value)'
                self.value_curve   =  self.curve_data[1, (np.abs(self.curve_data[0] - x)).argmin()]
                self.indice_curve   =  self.curve_data[0, (np.abs(self.curve_data[0] - x)).argmin()]
                self.index_curve   =  np.where(self.curve_data[0] == self.indice_curve)[0][0]
                self.curve_index_normed = self.index_curve / self.curve_data[1].size


    def get_elbow_error(self, x):
        self.set_tcp_position(x)
        self.set_value_curve(x)
        return [self.tcp_position_y - self.curve_index_normed  , self.tcp_position_z - self.value_curve]



    def get_shoulder_error(self, x):
        self.set_tcp_position(x)
        self.set_value_curve(x)
        return [self.curve_index_normed  - self.tcp_position_y, self.value_curve - self.tcp_position_z]




########################################### L E A R N I N G  - P O L A R #################################



    def get_shoulder_error_polar(self,x):
        target_pos = copy.copy(self.tcp.position)
        tcp_pos = copy.copy(self.tcp.position)
        shoulder = copy.copy(self.shoulder.position)

        if shoulder is None or target_pos is None:
            return 0.0
        else:
            self.set_value_curve(x)
            self.set_tcp_position(x)
            target_pos.z = self.value_curve + shoulder.z
            # Vektor berchnung
            vector_s_tcp = [tcp_pos.x - shoulder.x, tcp_pos.y - shoulder.y, tcp_pos.z - shoulder.z] # PQ  = Q-P
            vector_s_target = [target_pos.x - shoulder.x, target_pos.y - shoulder.y, target_pos.z - shoulder.z]
            dot_product = np.dot(vector_s_tcp, vector_s_target)
            cross_product = np.cross(vector_s_tcp, vector_s_target)
            length_vector_s_tcp = np.linalg.norm(vector_s_tcp)
            length_vector_s_target = np.linalg.norm(vector_s_target)
            sign = cross_product[0] / abs(cross_product[0])

            return sign * math.acos(dot_product / (length_vector_s_tcp * length_vector_s_target) )


    def get_elbow_error_polar(self,x):
        shoulder = copy.copy(self.shoulder.position)
        if shoulder is None:
            return 0.0
        else:
            self.set_value_curve(x)
            self.set_tcp_position(x)
            tcp_initial  = shoulder
            tcp_initial.y  = shoulder.y +0.3 # shoulder pos zu initial tcp pos umrechnen
            target_pos = copy.copy(shoulder)
            target_pos.z = self.value_curve + tcp_initial.z
            target_pos.y =  tcp_initial.y - self.curve_index_normed
            target_pos_vector = self.calc_vector(target_pos)
            target_pos_polar = self.calc_polar(target_pos_vector.x, target_pos_vector.y, target_pos_vector.z)
            return self.tcp.polar_pos[0] - target_pos_polar[0]

