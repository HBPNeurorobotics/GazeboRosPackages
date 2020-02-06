#!/usr/bin/env python

import numpy as np
import nengo

class Generate_curve(object):
    def __init__(self, n_points, amplitude, period, phase_shift, vertical_shift, do_print):
        self.n_points       = n_points
        self.amplitude      = amplitude
        self.period         = period
        self.phase_shift    = phase_shift
        self.vertical_shift = vertical_shift
        self.data           = self.generate_sin()
        if do_print:
            self.print_data()

    def generate_sin(self):
        u = np.arange(self.n_points)
        u = u / float((self.n_points)-1)
        sin_u = self.amplitude * np.sin (u * self.period + self.phase_shift ) +  self.vertical_shift
        sin_u = np.round(sin_u, 2)
        data = np.array([u, sin_u])
        return data

    def print_data(self):
        print
        indices_arrstr = np.char.mod('%f', self.data[0])
        indices = ", ".join(indices_arrstr)
        print 'Indices: ', indices
        print
        values_arrstr = np.char.mod('%f', self.data[1])
        values = ", ".join(values_arrstr)
        print 'Values: ', values

model = nengo.Network()
