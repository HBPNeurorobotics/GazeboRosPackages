#!/usr/bin/env python

import numpy as np
import nengo
import os
from time import gmtime, strftime

class GenerateCSV_data(object):
    def __init__(self, file_name ='test', do_print = False):
        self.timestamp = strftime('_%m_%d_%H_%M',  gmtime())
        self.file_name = str(file_name + self.timestamp + '.csv')
        if do_print:
            data_check = self.csv_load(self.file_name)
            print data_check

    def csv_save(self, t, x):
        if int(round( x[0], 0)) == 1:
            data = np.array([t, x[0]])
            with open(self.file_name, "a") as myfile:
                myfile.write( str(round( x[1], 2)) + ', ')

    def csv_load(self, file_name):
        data = np.loadtxt(file_name, delimiter=",")
        print 'loading'
        return data

model = nengo.Network()
