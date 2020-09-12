#!/usr/bin/env python

import math
import operator
from random import uniform

import nengo
import numpy as np

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


class Feedback(object):
    def __init__(self, sensor_joints = [], threshold = [], neuron_number = 21):
        self.sensor_joints = sensor_joints
        self.neuron_number = neuron_number # for visualization of position feedback
        self.arm = JointState()
        self.arm.name = rospy.get_param('~joint_names', ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint'])
        self.arm.position=[0.0 for i in range(len(self.arm.name))]
        self.arm.effort= [0.0 for i in range(len(self.arm.name))]
        self.avg = []
        self.threshold_mapping = np.array(
            [[600,    620,    650,    1000,   200,    1000],    # positiv threshold
            [-600,   -610,   -650,   -1000,  -170,   -1000]])   # negativ threshold
        if len(threshold) is not 0:
            for i in range (len(self.sensor_joints)):
                self.threshold_mapping[0][sensor_joints[i]] = threshold[0][i]
                self.threshold_mapping[1][sensor_joints[i]] = threshold[1][i]
        feedback_data_pub_topic = rospy.get_param('~feedback_data_pub_topic', '/feedback_data_pub')
        self.feedback_data_pub = rospy.Publisher(feedback_data_pub_topic, String, queue_size = 1)
        self.joint_states_topic = rospy.get_param('~joint_states_topic', '/joint_states')
        self.is_icub = rospy.get_param('~is_icub', False)
        self.last_used_joint_states = [0.0 for i in range(len(self.arm.name))]
        if self.is_icub:
            self.icub_sub = []
            for i in range(len(self.arm.name)):
                sub = rospy.Subscriber(self.arm.name[i], Float64, self.icub_callback, callback_args=i, queue_size=1)

    def icub_callback(self, msg, joint_index):
        self.arm.position[joint_index] = msg.data

    def callback(self, data):
        for i in range(len(self.arm.name)):
            for j in range(len(data.name)):
                if self.arm.name[i] == data.name[j]:
                    self.arm.position[i] = data.position[j]
                    self.arm.effort[i] = data.effort[j]

    def get_network_effort(self, label):
        def get_feedback_effort(t):
            if self.arm is None:
                return 0
            else:
                #Orginal: return self.arm.effort[self.sensor_joints]
                res = []
                for i in self.sensor_joints:
                    res.append(self.arm.effort[i])
                return res

        def get_threshold(x):
            for i in range(len(self.sensor_joints)):
                if x[i] > self.threshold_mapping[0][self.sensor_joints[i]] or x[i] < self.threshold_mapping[1][self.sensor_joints[i]]:
                    return 10
            return 0

        def get_avg( x):
            self.avg.append(x)
            if(len(self.avg) > 500):
                self.avg.pop(0)
            res = sum(self.avg) / float(len(self.avg))
            return x

        def get_threshi(x):
            if abs(x) > 50:
                return 1
            else:
                return 0

        sub = rospy.Subscriber(self.joint_states_topic, JointState, self.callback, queue_size=1)
        net = nengo.Network(label=label)
        with net:
            net.input = nengo.Node(get_feedback_effort)
           # net.ens = nengo.Ensemble(n_neurons=200, dimensions=1, radius=10) #' RADIUS "200
            #net.ens = nengo.Ensemble(n_neurons=150, dimensions=1)
            net.output = nengo.Node(size_in=1)

            #TRYOUT
            net.ens = nengo.Ensemble(n_neurons=150, dimensions=len(self.sensor_joints), neuron_type=nengo.Direct())
            net.ens_2 = nengo.Ensemble(n_neurons=150, dimensions=len(self.sensor_joints), neuron_type=nengo.Direct())
            nengo.Connection(net.input, net.ens)
            for i in range(len(self.sensor_joints)):
                nengo.Connection(net.ens[i], net.ens_2[i], function = get_avg)
            #nengo.Connection(net.ens_2, net.output, function = get_threshi)
            nengo.Connection(net.ens_2, net.output, function = get_threshold)
            #nengo.Connection(net.input, net.ens, function=get_threshold)
            #nengo.Connection(net.ens, net.output)
        return net

    def get_network_position(self, label, joint, max_val, min_val, noise = False):
        def get_feedback_position(x):
            curr_pos = self.arm.position[joint-1]
            res = math.degrees(curr_pos)
            #res = min(max_val-1, max(min_val+1, res)) # original
            res = min(max_val, max(min_val, res))
            to_pub = "[min,max]: [{:.2f}, {:.2f}], joint: {}, arm_pos: {:.2f}, deg: {:.2f} res: {:.2f}".format(min_val, max_val, joint, curr_pos, math.degrees(curr_pos), res)
            self.feedback_data_pub.publish(to_pub)
            self.last_used_joint_states[joint-1] = curr_pos
            return res

        def get_mean(x):
            res = abs(int((x + abs(min_val))/((max_val -min_val) / float(self.neuron_number-1))))

            return res

        def exit(x):
            threshold = [1 for i in range(self.neuron_number)]
            mean =get_mean(x)
            for i in range(self.neuron_number):
                d = abs(mean -i)
                if mean == i:
                    threshold[i] =  uniform(5, 1) *100
                else:
                    if noise:
                       # threshold[i] = uniform(-2/d, d/2)
                       threshold[i] = uniform(-2/d, d)
                    #else:
                    #    if (mean+1 == i or mean-1 == i):
                     #       threshold[i] =  uniform(-0.7, 0.5)
            newList = []
            for j in threshold:
                #newList.append(j * 100)
                newList.append(j * 0.8)
            return newList

        sub = rospy.Subscriber(self.joint_states_topic, JointState, self.callback, queue_size=1)
        net = nengo.Network(label=label)
        with net:

            net.input = nengo.Node(get_feedback_position)
            net.gauss_encoding = nengo.Ensemble(
                    label='gauss_encoding',
                    n_neurons=self.neuron_number,
                    dimensions=1,
                   # intercepts=[3.1 for i in range(self.neuron_number)],
                    intercepts=[0.9 for i in range(self.neuron_number)],
                    max_rates=[10 for i in range(self.neuron_number)],
                    encoders=[[1] for i in range(self.neuron_number)])
            nengo.Connection(net.input, net.gauss_encoding.neurons, function = exit)
            def get_index(x):
                # gets index with highest value
                index, value = max(enumerate(x), key=operator.itemgetter(1))
                #if no high value exist set to highest index (prevent weird behaviour)
                vals = [i for i in x if abs(i) > 0.2 ]
                if label is 'feedback_1_LR':
                    if len(vals) < 1:
                        index = self.neuron_number -1
                        '''
                        print '      '
                        print '###################'
                        print 'ind: ', index,
                        print 'val: ', value
                        print 'x: ', x
                        print 'vals: ', vals
                        '''
                return index

            net.ens_calc = nengo.Ensemble(label='ens_calc', n_neurons= 100, dimensions=self.neuron_number,  neuron_type=nengo.Direct())

            # without ensemble array: values go up to 15
            #nengo.Connection(net.gauss_encoding.neurons, net.ens_calc)

            # with ensemlbe array --> works very well with 10 neurons in gaussian encoding
            net.ens_array = nengo.networks.EnsembleArray(
                    n_neurons=10,
                    n_ensembles=self.neuron_number)
            nengo.Connection(net.gauss_encoding.neurons, net.ens_array.input)
            nengo.Connection(net.ens_array.output, net.ens_calc)#, transform= [5] * self.neuron_number)
            net.output = nengo.Node(size_in= 1)

            # new
            '''
            def smooth(x):
                tau=0.01
                dt = 1e-3
                return Lowpass(tau).filt(x, dt=dt, y0=0)
            net.ens_unsmoothed = nengo.Ensemble(label='ens_unsmoothed', n_neurons= 100, dimensions=1, neuron_type=nengo.Direct())
            nengo.Connection(net.ens_calc, net.ens_unsmoothed, function=get_index)

            nengo.Connection(net.ens_unsmoothed, net.output, function= smooth)
            '''

            # old
            nengo.Connection(net.ens_calc, net.output, function=get_index)
        return net
