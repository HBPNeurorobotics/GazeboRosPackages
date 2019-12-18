#!/usr/bin/env python
""" """

__author__ = 'Omer Yilmaz'

import rospy
from external_module_interface.external_module import ExternalModule


class Module2(ExternalModule):

    def __init__(self, module_name=None, steps=1):
        super(Module2, self).__init__(module_name, steps)
    
    def run_step(self):
        pass

    def share_module_data(self):
        self.module_data = [0, 0, 5.1]

if __name__ == "__main__":
    m = Module2(module_name='module_cerebellum', steps=8)
    rospy.spin()
