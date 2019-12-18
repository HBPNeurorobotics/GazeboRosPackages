"""
External Module base class for the modules added by the user. This class is used
for the synchronization with the Deterministic Closed Loop Engine. ExternalModule
provides the initialize, run_step and shutdown methods and ROS services attached
to them. These ROS services are triggered by the Deterministic Closed Loop Engine
ensuring the synchronization. You have to have a ros node before inheriting from
this class. Then the necessary ROS services are introduced automatically.
"""

__author__ = 'Omer Yilmaz'

import time
import rospy
from std_msgs.msg import String
from cle_ros_msgs.srv import Initialize, RunStep, Shutdown, \
                        InitializeResponse, RunStepResponse, ShutdownResponse
from iba_multimodule_experiment.srv import Registration, RegistrationRequest, RegistrationResponse, \
                        SetData, SetDataRequest, SetDataResponse, \
                        GetData, GetDataRequest, GetDataResponse


class ExternalModule(object):

    def __init__(self, module_name=None, steps=1):
        self.module_name = module_name
        rospy.init_node(module_name)
        print(self.module_name)
        self.initialize_service = rospy.Service('emi/' + self.module_name + '_module/initialize', Initialize, self.initialize_call)
        self.run_step_service = rospy.Service('emi/' + self.module_name + '_module/run_step', RunStep, self.run_step_call)
        self.shutdown_service = rospy.Service('emi/' + self.module_name + '_module/shutdown', Shutdown, self.shutdown_call)
        self.module_data = []
        self.module_id = 0
        self.n_steps = steps
        self.step = 0
        self.time = 0.0
        self.database_req = GetDataRequest()
        self.database_resp = GetDataResponse()

        rospy.wait_for_service('emi/manager_module/registration_service')
        self.registration_proxy = rospy.ServiceProxy('emi/manager_module/registration_service', Registration)
        resp = self.registration_proxy(RegistrationRequest(String(self.module_name), self.n_steps))
        self.module_id = resp.id

        rospy.wait_for_service('emi/manager_module/get_data_service')
        self.database_proxy = rospy.ServiceProxy('emi/manager_module/get_data_service', GetData)

        rospy.wait_for_service('emi/manager_module/set_data_service')
        self.update_database_proxy = rospy.ServiceProxy('emi/manager_module/set_data_service', SetData)

    def initialize_call(self, req):
        self.initialize()
        return InitializeResponse()

    def initialize(self):
        pass

    def run_step_call(self, req):
        self.time += 1.0
        for self.step in range(1, self.n_steps + 1):
            while True:
                self.database_resp = self.database_proxy(self.module_id, self.step-1)
                if self.database_resp.lock.data == False:
                    print "Module " + str(self.module_id) + " time: " + "{0:.2f}".format(self.time + float(self.step-1)/self.n_steps) + " step: " + str(self.step) + " started!"
                    break
                time.sleep(0.001)

            self.run_step()
            self.share_module_data_caller()

            while True:
                self.update_database_resp = self.update_database_proxy(self.module_id, self.step, self.module_data)
                if self.update_database_resp.lock.data == False:
                    print "Module " + str(self.module_id) + " time: " + "{0:.2f}".format(self.time + float(self.step)/self.n_steps) + " step: " + str(self.step) + " ended!"
                    break
                time.sleep(0.001)
        return RunStepResponse()

    def run_step(self):
        pass

    def shutdown_call(self, req):
        self.shutdown()
        self.initialize_service.shutdown()
        self.run_step_service.shutdown()
        return ShutdownResponse()

    def shutdown(self):
        pass

    def share_module_data_caller(self):
        self.module_data = []
        self.share_module_data()
        self.module_data.insert(0, self.step)

    def share_module_data(self):
        pass
