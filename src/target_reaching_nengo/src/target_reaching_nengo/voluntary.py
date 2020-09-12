#!/usr/bin/env python

#import time
#import timeit
from operator import add, mul, sub

import nengo
import numpy as np

from target_reaching_nengo import Motion

import rospy
from std_msgs.msg import String


class Voluntary(Motion):
    def __init__(self, slider, joints, start, end, label = 'voluntary', joint_mapping = None, neuron_number = 10, learning = False):
        super(self.__class__, self).__init__(joints, start, end, label, joint_mapping)
        self._slider = slider
        self._neuron_number = neuron_number
        self.learning = learning

        #korrekt fur n = 21
        #indices = np.array([ 0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.8, 0.85, 0.90, 0.95, 1.])
        #values  = np.array([ 0.,  0.01, 0.02, 0.05, 0.1,  0.15, 0.21, 0.27, 0.35, 0.42, 0.5,  0.58, 0.65, 0.73, 0.79, 0.85, 0.9, 0.95, 0.98, 0.99, 1.])
        #korrekt fur n = 40
        indices = np.array([0. , 0.03, 0.05,  0.08,  0.1,   0.13,  0.15,  0.18,  0.21,  0.23,  0.26,  0.28, 0.31,  0.33,  0.36,  0.38,  0.41,  0.44,  0.46,  0.49,  0.51,  0.54,  0.56,  0.59, 0.62,  0.64,  0.67,  0.69,  0.72,  0.74,  0.77,  0.79,  0.82, 0.85, 0.87, 0.9,  0.92, 0.95, 0.97, 1.])
        values =np.array([  0. , 0. ,  0.01,  0.01,  0.03,  0.04,  0.06,  0.08,  0.1,   0.13, 0.15, 0.18, 0.22, 0.25, 0.29, 0.32, 0.36, 0.4,0.44, 0.48, 0.52, 0.56, 0.6, 0.64, 0.68, 0.71, 0.75, 0.78, 0.82, 0.85, 0.87, 0.9,                          0.92, 0.94, 0.96, 0.97, 0.99, 0.99, 1.  , 1.])
        self._data =np.array([indices, values])

        self._stored_index = []
        topic_name = 'output_' + self._label
        self._puby= rospy.Publisher(topic_name , String, queue_size=1)

    def check_same_idx(self, error, index):
        if(len(self._stored_index) > 150):
            self._stored_index.pop(0)
        self._stored_index.append(index)

        if abs(error) > 0:
            one_third = self._stored_index[0: len(self._stored_index)/3 ]
            two_thirds = self._stored_index[0: 2*(len(self._stored_index)/3) ]

            if all(self._stored_index[0] == item for item in self._stored_index):
                return index + 6*(error / abs(error))
            elif all(self._stored_index[0] == item for item in two_thirds):
                return index + 4*(error / abs(error))
            elif all(self._stored_index[0] == item for item in one_third ):
                return index + 2*(error / abs(error))
        return index



        '''

numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
print numbers[3:6]


        A = [1,2,3,4,5,6]
    B = A[:len(A)/2]
    C = A[len(A)/2:]
    oneThird = self._stored_index[0: len(self._stored_index)/3 ]
    two thirds = self._stored_index[0: 2*(len(self._stored_index)/3) ]

        '''


    ''' ORGINAL
    def check_same_idx(self, error, index):
        if(len(self._stored_index) > 30):
            self._stored_index.pop(0)
        self._stored_index.append(index)
        if abs(error) > 0 and all(self._stored_index[0] == item for item in self._stored_index):
            return index + 2*(error / abs(error))
        else: return index
    '''

    def get_u_TR(self, x):
        delta  = (int)(round(x[0]))
        u_val = min(1.0, max(0.0, x[1] / self._neuron_number ))   #  (self._neuron_number - 1)
        u_indice = self._data[0, (np.abs(self._data[1] - u_val)).argmin()]
        u_index = np.where(self._data[0] == u_indice)[0]
        index = self.check_same_idx(delta, u_index)
        u_next = self._data[1, min(max(index + delta, 0), self._data[1].size - 1)]




        self._puby.publish(
             self._label + '     '+
            '  pos: '+  (str)(x[1]) +
            '  error: '+  (str)(delta)+
            '  u_val: '+ (str)(u_val) +
            '  u_indice: '+ (str)(u_indice)+
            '  u_index: '+ (str)(u_index)+
            '  index: '+ (str)(index)+
            '  u_next: '+ (str)(u_next))
        return u_next



    # this method returns the error and proprioceptive controlled voluntary network
    def get_network_TR(self, label):
        net = nengo.Network(label=label)
        with net:
            net.input = nengo.Node(size_in = 2)
            net.perception_proprioception = nengo.Ensemble(n_neurons=600, dimensions = 2, radius =10, neuron_type=nengo.Direct())
            net.u = nengo.Ensemble(n_neurons=200, dimensions = 1, neuron_type=nengo.Direct())
            net.output = nengo.Node(size_in = len(self._joints), size_out = len(self._joints))
            nengo.Connection(net.input[0], net.perception_proprioception[0])
            nengo.Connection(net.input[1], net.perception_proprioception[1])#, transform = 10 / self.neuron_number)

            nengo.Connection(net.perception_proprioception, net.u, function = self.get_u_TR)#, transform= 1.2)
            #net.debug = nengo.Node(size_in = 1)
            #nengo.Connection(net.perception_proprioception, net.debug , function = self.get_u_TR)

            nengo.Connection(net.u, net.output, function=self.map_voluntary)
            #if len(self._joints) > 1:
            #    nengo.Connection(net.u, net.output[1], synapse=0.01, function=self.map_voluntary)
        return net

    # calculation for synapse for motions with 2 joints, so they move aligned
    def get_dif(self):
        prim_size_0 = abs(self._end[0] - self._start[0])
        prim_size_1 = abs(self._end[1] - self._start[1])
        res = abs(prim_size_0 - prim_size_1)
        #print ' prim_size_0: ',  prim_size_0, ' prim_size_1',  prim_size_1 , '  RES: ', res, ' res 2: ', res * 0.75
        return res



    # this method returns the slider controlled voluntary network
    def get_network_slider(self, label, inverted_reflex = False, factor = 1):
        net = nengo.Network(label=label)
        with net:

            net.input = nengo.Node(size_in = 1)
            net.ens = nengo.Ensemble(n_neurons=200, dimensions = 1, neuron_type=nengo.Direct(),  label = 'u')
            nengo.Connection(net.input, net.ens)
            net.output = nengo.Node(size_in = len(self._joints))
            if inverted_reflex:
                def reflex(x):
                    if x[1] < 0.1: return self.map_voluntary(x[0])
                    else: return self.map_voluntary(0.2)


                net.u = nengo.Ensemble(n_neurons=200, dimensions = 2, neuron_type=nengo.Direct(), label = 'f(u)')
                nengo.Connection(net.ens, net.u[0], function = self.get_u)
                #nengo.Connection(net.u, net.output, function=self.map_voluntary)
                nengo.Connection(net.u, net.output, function=reflex)
                '''
                def invert(x):
                    return (-(self.get_u(x)-0.5) +0.5) * factor

                def map_inv_u(x):
                    if x[2] < 0.5: return self.map_voluntary(x[0])
                    else: return self.map_voluntary(x[1])

                net.u = nengo.Ensemble(n_neurons=400, dimensions = 3, neuron_type=nengo.Direct(), label = 'f(u)')
                nengo.Connection(net.ens, net.u[0], function = self.get_u)
                nengo.Connection(net.ens, net.u[1], function = invert)
                nengo.Connection(net.u, net.output, function=map_inv_u)
                '''
            else:
                net.u = nengo.Ensemble(n_neurons=200, dimensions = 1, neuron_type=nengo.Direct(), label = 'f(u)')
                nengo.Connection(net.ens, net.u, function = self.get_u)
                nengo.Connection(net.u, net.output, function=self.map_voluntary)
        return net







    # this method returns the slider controlled voluntary network
    def get_network_learn(self, label):
        net = nengo.Network(label=label)

        with net:
            net.input = nengo.Node(size_in = 1)
            net.ens = nengo.Ensemble(n_neurons=200, dimensions = 1, neuron_type=nengo.Direct(), label='u')
            net.u_global = nengo.Ensemble(n_neurons=200, dimensions = 1, label='f(u) global')

            net.u_local = nengo.Ensemble(n_neurons=400, dimensions = 2, label='f(u) local') # first shoulder, second elbow
            #net.u_shoulder = nengo.Ensemble(n_neurons=200, dimensions = 1)#, neuron_type=nengo.Direct())
           # net.u_elbow = nengo.Ensemble(n_neurons=200, dimensions = 1)#, neuron_type=nengo.Direct())

            net.output = nengo.Node(size_in = len(self._joints))

            nengo.Connection(net.input, net.ens)
            nengo.Connection(net.ens, net.u_global, function = self.get_u)

            nengo.Connection(net.u_local[0], net.output[0], function=self.map_voluntary_1)
            nengo.Connection(net.u_local[1], net.output[1], function=self.map_voluntary_2)
        return net


        '''
        #ORGINAL
        with net:
            net.input = nengo.Node(size_in = 1)
            net.ens = nengo.Ensemble(n_neurons=200, dimensions = 1, neuron_type=nengo.Direct())
            net.u_global = nengo.Ensemble(n_neurons=200, dimensions = 1)#, neuron_type=nengo.Direct())
            net.u_shoulder = nengo.Ensemble(n_neurons=200, dimensions = 1)#, neuron_type=nengo.Direct())
            net.u_elbow = nengo.Ensemble(n_neurons=200, dimensions = 1)#, neuron_type=nengo.Direct())
            net.output = nengo.Node(size_in = len(self._joints))
            nengo.Connection(net.input, net.ens)
            nengo.Connection(net.ens, net.u_global, function = self.get_u)
            nengo.Connection(net.u_shoulder, net.output[0], function=self.map_voluntary_1)
            nengo.Connection(net.u_elbow, net.output[1], function=self.map_voluntary_2)
        return net
        '''
