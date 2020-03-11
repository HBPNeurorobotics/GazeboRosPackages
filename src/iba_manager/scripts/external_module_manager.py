#!/usr/bin/env python
"""
Manages instantiated modules. Handles step calling and data synchronization between modules
"""

__author__ = 'Omer Yilmaz'

import math
import time
import rospy
from std_msgs.msg import Bool
from external_module_interface.external_module import ModuleState
from iba_manager.srv import Initialize, InitializeResponse, \
    RunStep, RunStepResponse, Shutdown, ShutdownResponse, Registration, \
    GetData, GetDataResponse, SetData, SetDataResponse
from threading import Lock
import copy


class ModuleManagerState(ModuleState):
    def __init__(self):
        super(ModuleManagerState, self).__init__(1)

        # Number of registered modules
        self.num_modules = 0

        # Array with number of steps each module will execute per CLE timestep
        self.module_steps_per_cle_loop = []

        # Array with module step size, e.g. at which ModuleManager steps should a module be allowed to run
        self.module_step_size = []


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

        # Set up services for initialization, stepping, and shutdown
        self._initialize_service = rospy.Service('emi/manager_module/initialize', Initialize, self.initialize_callback)
        self._run_step_service = rospy.Service('emi/manager_module/run_step', RunStep, self.run_step_callback)
        self._shutdown_service = rospy.Service('emi/manager_module/shutdown', Shutdown, self.shutdown_callback)

        # Set up service for modules to register themselves with this manager
        self._registration_service = rospy.Service('emi/manager_module/registration_service', Registration, self.registration_callback)
        self._registration_lock = Lock()

        # Set up service for modules to retrieve synchronized data at start of their run_step
        self._get_data_service = rospy.Service('emi/manager_module/get_data_service', GetData, self.get_data_callback)

        # Set up service for modules to send data at end of their run_step
        self._set_data_service = rospy.Service('emi/manager_module/set_data_service', SetData, self.set_data_callback)

        # Initialize synchronization data
        self._synced_data = GetDataResponse(lock=Bool(False))

        self._future_data_lock = Lock()
        self._future_data = {}           # Dict that will be filled with data from completed module run_steps, mapped to the corresponding module ID

        # Initialize IBA params
        self._state = ModuleManagerState()

        # Initialize starting_modules array
        self._starting_modules_lock = Lock()
        self._starting_modules = []      # Array with IDs of all modules that should start at this step

    def registration_callback(self, req):
        """Set up state variables for every module"""
        with self._registration_lock:
            self._state.num_modules += 1

            # Round module_steps up to the nearest power of two
            module_steps = self._state.round_step_up(req.steps)
            self._state.module_steps_per_cle_loop.append(module_steps)

            return self._state.num_modules-1

    def initialize_callback(self, req):
        """
        Initialize manager module. Set up variable for each service, indicating both the requested amount of steps
        as well as the current step
        """
        # No more registration should be possible once the initialization method has been called
        self._registration_service.shutdown()

        # Set number of steps the ModuleManager should execute per CLE cycle. As the manager synchronizes data
        # between the modules, it must run at the maximum frequency set by the registered modules.
        self._state.steps_per_cle_cycle = max(self._state.module_steps_per_cle_loop)

        # Set step size of all modules
        self._state.module_step_size = [self._state.steps_per_cle_cycle/module_steps
                                        for module_steps in self._state.module_steps_per_cle_loop]

        return InitializeResponse()

    def run_step_callback(self, req):
        """Single CLE cycle. Execute the manager's run_step method max_step times"""
        for step in range(0, self._state.steps_per_cle_cycle):
            self.run_step(step)
        return RunStepResponse(status=True)

    def run_step(self, step):
        """
        At every run_step, perform data synchronization between modules.
        Wait for data from any modules that have finished their step and send out data to any module that will now
        start their step. See get_data_function and set_data_function for details about synchronization
        """
        self._state.cur_step = step

        # Setup which modules should start this step
        with self._starting_modules_lock:
            self._starting_modules = []
            for module_id in range(0, self._state.num_modules):
                if step % self._state.module_step_size[module_id] == 0:
                    self._starting_modules.append(module_id)

        # Setup which modules should finish before next step
        finishing_modules = []
        for module_id in range(0, self._state.num_modules):
            if (step+1) % self._state.module_step_size[module_id] == 0:
                finishing_modules.append(module_id)

        # Wait for all modules to start. This is done via the modules calling get_data_callback.
        # The callback will remove the ID of all started modules from the array. Once the array is empty,
        # all modules are running
        while self._starting_modules:
            time.sleep(0.001)

        # Wait for all modules to finish. This is done via the modules calling set_data_callback
        while finishing_modules:
            with self._future_data_lock:
                for module_id in reversed(finishing_modules):
                    if module_id in self._future_data:
                        # Move any data that should become available next step from future_data to synced_data
                        self._synced_data.__setattr__("m" + str(module_id), self._future_data[module_id])

                        self._future_data.pop(module_id)
                        finishing_modules.remove(module_id)

            time.sleep(0.001)

    def shutdown_callback(self, req):
        """Shutdown ServiceProxies"""
        self._initialize_service.shutdown()
        self._run_step_service.shutdown()
        self._registration_service.shutdown()
        self._set_data_service.shutdown()
        self._get_data_service.shutdown()
        return ShutdownResponse(status=True)

    def get_data_callback(self, req):
        """
        Service Callback used by modules to collect data from the manager. Will keep module locked until all data
        from previous run_steps has been collected
        """
        with self._starting_modules_lock:
            # Check whether the requesting module can start this step
            if req.id not in self._starting_modules:
                # ID not in starting_modules, keep this module from executing
                self._synced_data.lock.data = True
            else:
                # ID in starting_modules, allow module execution
                self._starting_modules.remove(req.id)
                self._synced_data.lock.data = False

            return copy.deepcopy(self._synced_data)

    def set_data_callback(self, req):
        """
        Service Callback used by modules to send data to the ModuleManager
        """
        with self._future_data_lock:
            self._future_data[req.id] = req.m

        return SetDataResponse(lock=Bool(False))


if __name__ == "__main__":
    mm = ManagerModule()
    rospy.spin()
