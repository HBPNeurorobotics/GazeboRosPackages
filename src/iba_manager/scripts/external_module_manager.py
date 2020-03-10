#!/usr/bin/env python
"""
Manages instantiated modules. Handles step calling and data synchronization between modules
"""

__author__ = 'Omer Yilmaz'

import math
import time
import rospy
from std_msgs.msg import Bool
from iba_manager.srv import Initialize, InitializeResponse, \
    RunStep, RunStepResponse, Shutdown, ShutdownResponse, Registration, \
    GetData, GetDataResponse, SetData, SetDataResponse


class ManagerModule(object):
    """
    Manages instantiated modules. Handles step calling and data synchronization between modules
    """

    def __init__(self):
        """
        Sets up services to handle module stepping and data synchronization
        """
        module_name = "manager"
        rospy.init_node(module_name)
        self.initialize_service = rospy.Service('emi/manager_module/initialize', Initialize, self.initialize_call)
        self.run_step_service = rospy.Service('emi/manager_module/run_step', RunStep, self.run_step_call)
        self.shutdown_service = rospy.Service('emi/manager_module/shutdown', Shutdown, self.shutdown_call)

        # Set up service for modules to register themselves with this manager
        self.registration_service = rospy.Service('emi/manager_module/registration_service', Registration, self.registration_function)

        # Set up service for modules to retrieve synchronized data at start of their run_step
        self.get_data_service = rospy.Service('emi/manager_module/get_data_service', GetData, self.get_data_function)

        # Set up service for modules to send data at end of their run_step
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
        """
        Initialize manager module. Set up variable for each service, indicating both the requested amount of steps
        as well as the current step
        """
        self.max_steps = max(self.module_steps.itervalues())[0]
        self.max_steps = int(2**math.ceil(math.log(self.max_steps, 2)))
        for key in self.module_steps.iterkeys():
            self.module_steps[key][1] = self.max_steps / int(2**math.ceil(math.log(float(self.module_steps[key][0]), 2)))
        self.initialize()
        return InitializeResponse()

    def initialize(self):
        pass

    def run_step_call(self, req):
        """Run this module at every step"""
        self.time += 1.0
        for step in range(1, self.max_steps + 1):
            self.run_step(step)
        return RunStepResponse()

    def run_step(self, step):
        """
        At every run_step, perform data synchronization between modules.
        Wait for data from any modules that have finished their step and send out data to any module that will now
        start their step. See get_data_function and set_data_function for details
        """
        self.get_data_list = []
        self.set_data_list = []

        # Set up array to track which modules should receive data and which modules will send data
        for k, v in self.module_steps.items():
            # Wait for modules that will complete to send their data
            self.module_steps[k][2] = (step-1) % self.module_steps[k][1]
            if self.module_steps[k][2] == 0:
                self.get_data_list.append(k)

            # Wait for modules that will start to request data
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
                time.sleep(0.001)
                break

    def shutdown_call(self, req):
        """Shutdown ServiceProxies"""
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
        """Set up state variables for every module"""
        self.module_count = self.module_count + 1
        self.module_steps[self.module_count] = [req.steps, 1, -1, -1, 0]
        return self.module_count

    def get_data_function(self, req):
        """
        Service Callback used by modules to request data from the manager. Will keep module locked until all data
        from previous run_steps has been collected
        """
        self.module_steps[req.id][4] = req.step * self.module_steps[req.id][1]
        if req.id in self.get_data_list and self.module_steps[req.id][4]+1 == self.step:
            self.get_data_list.remove(req.id)
            self.get_data_resp.lock.data = False

            if self.step == self.max_steps and all([getattr(self.get_data_resp, "m" + str(i))[0] == self.module_steps[i][0] for i in range(1, self.module_count+1)]):
                for i in range(1, self.module_count+1):
                    getattr(self.get_data_resp, "m" + str(i))[0] = 0
                self.get_data_resp.lock.data = False

        else:
            self.get_data_resp.lock.data = True

        return self.get_data_resp

    def set_data_function(self, req):
        """
        Service Callback used by modules to send data to module. Module will wait to process data until the data is
        requested by other modules. During this time, the module will remain in a locked state, preventing further
        execution
        """
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
