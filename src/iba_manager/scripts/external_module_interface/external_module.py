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
import math
from threading import Lock
from std_msgs.msg import String
from iba_manager.srv import Initialize, InitializeResponse, RunStep, RunStepResponse, Shutdown, ShutdownResponse, \
                        Registration, RegistrationRequest, SetData, \
                        GetData, GetDataRequest, GetDataResponse


class ModuleState(object):
    def __init__(self, requested_steps=1):
        self.requested_steps = requested_steps
        self.steps_per_cle_cycle = self.round_step_up(requested_steps)

        self.cur_step = 0
        self.is_locked = True

    def round_step_up(self, step):
        cur_step = 1
        while cur_step < step:
            cur_step *= 2

        return cur_step


class ExternalModule(object):
    """
    IBA Module class. Registers itself with the ExternalModuleManager at startup and manages data sending and receiving.
    At every step, this class will first receive synchronized data from ExternalModuleManager. Secondly, the user code,
    available via the run_step method of a derived class, will be executed. Lastly, user data will be synchronized with
    the ExternalModuleManager. Other methods a user can override are initialize, share_module_data, and shutdown.
    """

    def __init__(self, module_name=None, steps=1):
        """Sets up services required for communication with the CLE as well as the ExternalModuleManager"""
        self.module_name = module_name
        self._state = ModuleState(steps)

        # Start ROS Node
        rospy.init_node(module_name)

        # Set up initialization, step, and shutdown services
        # TODO: Change the service naming convention. Currently, it is using only the module_name, which will
        #  cause problems if multiple modules have the same name or the name of the manager. If errors are encountered
        #  with the naming convention, consider changing it. For that, also take into account the CLE ExternalManager
        #  and ExternalModuleManager class to look for the module services at the new location
        self._initialize_service = rospy.Service('emi/' + self.module_name + '_module/initialize', Initialize, self.initialize_call)
        self._run_step_service = rospy.Service('emi/' + self.module_name + '_module/run_step', RunStep, self.run_step_call)
        self._shutdown_service = rospy.Service('emi/' + self.module_name + '_module/shutdown', Shutdown, self.shutdown_call)

        # Initialize synchronization variables
        self.module_data = []
        self._database_req = GetDataRequest()
        self._database_resp = GetDataResponse()

        self.run_step_lock = Lock()

        # Register module with ExternalModuleManager
        module_manager_service = 'emi/manager_module/registration_service'
        rospy.wait_for_service(module_manager_service)
        registration_proxy = rospy.ServiceProxy(module_manager_service, Registration)
        resp = registration_proxy(RegistrationRequest(String(self.module_name), self._state.steps_per_cle_cycle))
        self.module_id = resp.id

        # Set up data synchronization services
        rospy.wait_for_service('emi/manager_module/get_data_service')
        self._manager_get_data_proxy = rospy.ServiceProxy('emi/manager_module/get_data_service', GetData)

        rospy.wait_for_service('emi/manager_module/set_data_service')
        self._manager_set_data_proxy = rospy.ServiceProxy('emi/manager_module/set_data_service', SetData)

    def initialize_call(self, req):
        """Calls user-defined initialize method"""
        self.initialize()
        return InitializeResponse(status=True)

    def initialize(self):
        """Initialize method. Run at startup. Can be overridden by the user"""
        pass

    def run_step_call(self, req):
        """
        Executes n_steps during a single CLE cycle. Will first retrieve synchronized data
        from the ExternalModuleManager, then execute user code, and lastly send data back to the Manager
        """
        for self._state.cur_step in range(0, self._state.steps_per_cle_cycle):
            # Retrieve synchronized data from the ExternalModuleManager
            manager_sync_data_resp = self._manager_get_data_proxy.call(self.module_id, self._state.cur_step)
            while manager_sync_data_resp.lock.data is True:
                # ExternalModuleManager will keep module locked until all modules have finished the previous run_step.
                # Once all data is available, the manager will set the lock flag to False. Continue polling manager
                # until that happens
                time.sleep(0.001)
                manager_sync_data_resp = self._manager_get_data_proxy.call(self.module_id, self._state.cur_step)

            # Call user code
            self.run_step()
            self.share_module_data_call()

            # Send module data to ExternalModuleManager
            self._manager_set_data_proxy(self.module_id, self._state.cur_step + 1, self.module_data)

        return RunStepResponse(status=True)

    def run_step(self):
        """Step method. Runs every iteration step. Can be overridden by user"""
        pass

    def shutdown_call(self, req):
        """Calls user-defined shutdown method"""
        self.shutdown()

        self._initialize_service.shutdown()
        self._run_step_service.shutdown()

        return ShutdownResponse(status=True)

    def shutdown(self):
        """Shutdown method. Can be overridden by user"""
        pass

    def share_module_data_call(self):
        """Calls user-defined synchronization method and prepares module data for sending to the manager"""
        self.share_module_data()

    def share_module_data(self):
        """Data sending method. Can be used to customize data sending outside of the run_step method if required"""
        pass
