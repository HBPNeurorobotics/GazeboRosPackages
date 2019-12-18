#!/usr/bin/env python
"""
"""

__author__ = 'Omer Yilmaz'

import math
import time
import numpy as np
import rospy
from std_msgs.msg import String, Bool
from cle_ros_msgs.srv import Initialize, RunStep, Shutdown, \
                        InitializeResponse, RunStepResponse, ShutdownResponse
from iba_multimodule_experiment.srv import Registration, RegistrationRequest, RegistrationResponse, \
                        GetData, GetDataRequest, GetDataResponse, \
                        SetData, SetDataRequest, SetDataResponse


class ManagerModule():

    def __init__(self):
        module_name = "manager"
        rospy.init_node(module_name)
        self.initialize_service = rospy.Service('emi/manager_module/initialize', Initialize, self.initialize_call)
        self.run_step_service = rospy.Service('emi/manager_module/run_step', RunStep, self.run_step_call)
        self.shutdown_service = rospy.Service('emi/manager_module/shutdown', Shutdown, self.shutdown_call)
        self.registration_service = rospy.Service('emi/manager_module/registration_service', Registration, self.registration_function)
        self.get_data_service = rospy.Service('emi/manager_module/get_data_service', GetData, self.get_data_function)
        self.set_data_service = rospy.Service('emi/manager_module/set_data_service', SetData, self.set_data_function)
        self.get_data_resp = GetDataResponse(Bool(False), [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0])
        self.set_data_resp = SetDataResponse(Bool(True))
        self.max_steps = 1
        self.step = 0
        self.time = 0.0
        self.module_count = 0
        self.module_steps = {}
        self.set_data_list = []
        self.get_data_list = []

    def initialize_call(self, req):
        self.max_steps = max(self.module_steps.itervalues())[0]
        self.max_steps = int(2**math.ceil(math.log(self.max_steps, 2)))
        for key in self.module_steps.iterkeys():
            self.module_steps[key][1] = self.max_steps / int(2**math.ceil(math.log(float(self.module_steps[key][0]), 2)))
        self.initialize()
        return InitializeResponse()

    def initialize(self):
        pass

    def run_step_call(self, req):
        self.time += 1.0
        for step in range(1, self.max_steps + 1):
            self.run_step(step)
        return RunStepResponse()

    def run_step(self, step):
        self.get_data_list = []
        self.set_data_list = []
        for k, v in self.module_steps.items():
            self.module_steps[k][2] = (step-1) % self.module_steps[k][1]
            if self.module_steps[k][2] == 0:
                self.get_data_list.append(k)
            self.module_steps[k][3] = step % self.module_steps[k][1]
            if self.module_steps[k][3] == 0:
                self.set_data_list.append(k)

        self.step = step
        while True:
            if not self.get_data_list:
                time.sleep(0.001)
                break

        while True:
            if not self.set_data_list:
                #print self.set_data_list
                time.sleep(0.001)
                break

    def shutdown_call(self, req):
        self.shutdown()
        self.initialize_service.shutdown()
        self.run_step_service.shutdown()
        self.registration_service.shutdown()
        self.set_data_service.shutdown()
        self.get_data_service.shutdown()
        return ShutdownResponse()

    def shutdown(self):
        pass

    def registration_function(self, req):
        self.module_count = self.module_count + 1
        self.module_steps[self.module_count] = [req.steps, 1, -1, -1, 0]
        return self.module_count

    def get_data_function(self, req):
        self.module_steps[req.id][4] = req.step * self.module_steps[req.id][1]
        if req.id in self.get_data_list and self.module_steps[req.id][4]+1 == self.step:
            self.get_data_list.remove(req.id)
            self.get_data_resp.lock.data = False


            # if self.step == self.max_steps and self.get_data_resp.m1[0] == self.module_steps[1][0] and self.get_data_resp.m2[0] == self.module_steps[2][0] and self.get_data_resp.m3[0] == self.module_steps[3][0]:
                # self.get_data_resp.m1[0] = 0
                # self.get_data_resp.m2[0] = 0
                # self.get_data_resp.m3[0] = 0
                # return Bool(False), [self.module_steps[1][0]], [self.module_steps[2][0]], [self.module_steps[4][0]]
            if self.step == self.max_steps and all([getattr(self.get_data_resp, "m" + str(i))[0] == self.module_steps[i][0] for i in range(1, self.module_count+1)]):
                for i in range(1, self.module_count+1):
                    getattr(self.get_data_resp, "m" + str(i))[0] = 0
                self.get_data_resp.lock.data = False

        else:
            self.get_data_resp.lock.data = True

        return self.get_data_resp

    def set_data_function(self, req):
        self.module_steps[req.id][4] = req.step * self.module_steps[req.id][1]
        if req.id in self.set_data_list and self.module_steps[req.id][4] == self.step and not self.get_data_list:
            self.set_data_list.remove(req.id)
            self.set_data_resp.lock.data = False

            for i in range(1, self.module_count+1):
                if req.id == i:
                    setattr(self.get_data_resp, "m" + str(i), list(req.m))
                    break

        else:
            self.set_data_resp.lock.data = True

        return self.set_data_resp


if __name__ == "__main__":
    mm = ManagerModule()
    rospy.spin()
