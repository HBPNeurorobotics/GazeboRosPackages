#!/usr/bin/env python

import nengo
import numpy as np
import math
import rospy

from target_reaching_nengo import Voluntary, Base_network, Feedback, Error
from target_reaching_common import GenerateCSV_data

class Main_TR_CL:
    def __init__(self):
        arm_1_joint_cmd_pos_name = rospy.get_param('~arm_1_joint_cmd_pos_name', '/hbp/arm_1_joint/cmd_pos')
        arm_2_joint_cmd_pos_name = rospy.get_param('~arm_2_joint_cmd_pos_name', '/hbp/arm_2_joint/cmd_pos')
        arm_3_joint_cmd_pos_name = rospy.get_param('~arm_3_joint_cmd_pos_name', '/hbp/arm_3_joint/cmd_pos')
        arm_3_joint_index = rospy.get_param('~arm_3_joint_index', 3)
        voluntary_joints = [[arm_2_joint_cmd_pos_name],
                            [arm_1_joint_cmd_pos_name],
                            [arm_2_joint_cmd_pos_name,
                             arm_3_joint_cmd_pos_name]]
        base_network = Base_network(voluntary_joints = voluntary_joints, use_stim = False,
                                    arm_1_joint_cmd_pos_name = arm_1_joint_cmd_pos_name,
                                    arm_2_joint_cmd_pos_name = arm_2_joint_cmd_pos_name,
                                    arm_3_joint_cmd_pos_name = arm_3_joint_cmd_pos_name)
        neuron_number = 21

        up_down_lower_limit = rospy.get_param('~up_down_lower_limit', [-2.8])
        up_down_upper_limit = rospy.get_param('~up_down_upper_limit', [2.8])
        left_right_lower_limit = rospy.get_param('~left_right_lower_limit', [-2.8])
        left_right_upper_limit = rospy.get_param('~left_right_upper_limit', [2.8])
        near_far_lower_limit = rospy.get_param('~near_far_lower_limit', [-2.8, -2.8])
        near_far_upper_limit = rospy.get_param('~near_far_upper_limit', [2.8, 2.8])

        up_down = Voluntary(slider = 0, joints = voluntary_joints[0], start = up_down_lower_limit,
                            end = up_down_upper_limit, label = 'up_down', neuron_number = neuron_number)
        left_right = Voluntary(slider = 1, joints = voluntary_joints[1], start = left_right_lower_limit,
                               end = left_right_upper_limit, label = 'left_right', neuron_number = neuron_number)
        near_far = Voluntary(slider = 2, joints = voluntary_joints[2], start = near_far_lower_limit,
                             end = near_far_upper_limit,  label = 'near_far', neuron_number = neuron_number)

        err = rospy.get_param('~error_threshold', 0.05)
        self.error = Error(subject_name = 'target_reaching_subject', threshold = [ [-err, err],  [-err, err], [-err, err]])

        base_network.feedback = Feedback(neuron_number = neuron_number)
        csv_x = GenerateCSV_data(file_name ='x')
        csv_y = GenerateCSV_data(file_name ='y')
        csv_z = GenerateCSV_data(file_name ='z')
        csv_subject_x = GenerateCSV_data(file_name ='subject_x')
        csv_subject_y = GenerateCSV_data(file_name ='subject_y')
        csv_subject_z = GenerateCSV_data(file_name ='subject_z')
        csv_error_LR = GenerateCSV_data(file_name ='left_right')
        csv_error_UD = GenerateCSV_data(file_name ='up_down')
        csv_error_NF = GenerateCSV_data(file_name ='near_far')

        self.model = nengo.Network()
        with self.model:

            net = base_network.get_network('net')
            with net:
                error_node = nengo.Node(self.get_val)
                def kill_error(x):
                    if x==1:
                        return 1.
                    else:
                        return 0.
                def mult(x):
                    return x[0] * x[1] * 2

                #VOLUNTARY_1 : fern nah   --> fehler r (0)
                net_near_far = near_far.get_network_TR(label= 'near_far')
                nengo.Connection(error_node[0], net_near_far.input[0], function=base_network.set_error_near_far)

                nengo.Connection(net_near_far.output[0], net.f_u[near_far._slider])
                nengo.Connection(net_near_far.output[1], net.f_u[near_far._slider+1])

                #VOLUNTARY_2 : hoch runter  --> fehler theta(1)
                net_up_down = up_down.get_network_TR(label= 'up_down')
                nengo.Connection(error_node[1], net_up_down.input[0], function=base_network.set_error_up_down)
                nengo.Connection(net_up_down.output, net.f_u[up_down._slider])

                #VOLUNTARY_2 : left_right  --> fehler phi(2)
                net_left_right = left_right.get_network_TR(label= 'left_right')
                nengo.Connection(error_node[2], net_left_right.input[0], function=base_network.set_error_left_right)
                nengo.Connection(net_left_right.output, net.f_u[left_right._slider])

                #Propioception
                joint1_min_val = math.degrees(np.min(left_right_lower_limit))
                joint1_max_val = math.degrees(np.max(left_right_upper_limit))
                joint2_min_val = math.degrees(np.min([up_down_lower_limit[0], near_far_lower_limit[0]]))
                joint2_max_val = math.degrees(np.max([up_down_upper_limit[0], near_far_upper_limit[0]]))
                joint3_min_val = math.degrees(np.min(near_far_lower_limit[1]))
                joint3_max_val = math.degrees(np.max(near_far_upper_limit[1]))

                # TODO: change from degrees to radians if possible
                net_feedback_joint3_NF = base_network.feedback.get_network_position(label= 'FB: near_far', joint= arm_3_joint_index, max_val= joint3_max_val, min_val= joint3_min_val)
                net_feedback_joint2_HR_NF = base_network.feedback.get_network_position(label= 'FB: up_down and near_far' , joint= 2, max_val= joint2_max_val, min_val= joint2_min_val)
                net_feedback_joint1_LR = base_network.feedback.get_network_position(label= 'FB: left_right', joint= 1, max_val= joint1_max_val, min_val= joint1_min_val)

                nengo.Connection(net_feedback_joint3_NF.output, net_near_far.input[1])
                nengo.Connection(net_feedback_joint2_HR_NF.output, net_up_down.input[1])
                nengo.Connection(net_feedback_joint1_LR.output, net_left_right.input[1])

                # plot x
                net.do_save = nengo.Node(0)
                #TCP
                save_x = nengo.Node(csv_x.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_x[0])
                net.tcp_x_node = nengo.Node(self.get_tcp_pos_x)
                nengo.Connection(net.tcp_x_node, save_x[1])

                save_y = nengo.Node(csv_y.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_y[0])
                net.tcp_y_node = nengo.Node(self.get_tcp_pos_y)
                nengo.Connection(net.tcp_y_node, save_y[1])

                save_z = nengo.Node(csv_z.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_z[0])
                net.tcp_z_node = nengo.Node(self.get_tcp_pos_z)
                nengo.Connection(net.tcp_z_node, save_z[1])

                #subject
                save_subject_x = nengo.Node(csv_subject_x.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_subject_x[0])
                net.subject_x_node = nengo.Node(self.get_subject_pos_x)
                nengo.Connection(net.subject_x_node, save_subject_x[1])

                save_subject_y = nengo.Node(csv_subject_y.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_subject_y[0])
                net.subject_y_node = nengo.Node(self.get_subject_pos_y)
                nengo.Connection(net.subject_y_node, save_subject_y[1])

                save_subject_z = nengo.Node(csv_subject_z.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_subject_z[0])
                net.subject_z_node = nengo.Node(self.get_subject_pos_z)
                nengo.Connection(net.subject_z_node, save_subject_z[1])


                # ERROR

                save_error_LR = nengo.Node(csv_error_LR.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_error_LR[0])
                nengo.Connection(error_node[2], save_error_LR[1])

                save_error_UD = nengo.Node(csv_error_UD.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_error_UD[0])
                nengo.Connection(error_node[1], save_error_UD[1])

                save_error_NF = nengo.Node(csv_error_NF.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_error_NF[0])
                nengo.Connection(error_node[0], save_error_NF[1])

    def get_val(self, t):
        return self.error.get_val(t)

    def get_tcp_pos_x(self, t):
        return self.error.tcp.position.x

    def get_tcp_pos_y(self, t):
        return self.error.tcp.position.y

    def get_tcp_pos_z(self, t):
        return self.error.tcp.position.z

    def get_subject_pos_x(self, t):
        return self.error.subject.position.x

    def get_subject_pos_y(self, t):
        return self.error.subject.position.y

    def get_subject_pos_z(self, t):
        return self.error.subject.position.z

    def get_Network(self):
        return self.model
