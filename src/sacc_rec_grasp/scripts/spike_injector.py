#!/usr/bin/env python
from spynnaker8.external_devices import SpynnakerLiveSpikesConnection
from thread import allocate_lock
import rospy
from dvs_msgs.msg import EventArray
import sys
from sacc_rec_grasp.srv import DoSpikeInjection, DoSpikeInjectionResponse, DoStopSpikeInjection, \
    DoStopSpikeInjectionResponse
import numpy as np
import pdb


class SpikeInjector(object):

    def __init__(self):
        print("INJECTOR INIT")
        self.lock = allocate_lock()
        self.inject_spikes = False
        self.event_sub = rospy.Subscriber('/dvs_right/events', EventArray, self.event_callback,
                                          callback_args='/dvs_right/events')
        self.live_spikes_connection = SpynnakerLiveSpikesConnection(receive_labels=None, local_port=19996,
                                                                    send_labels=["spike_sender"])

        self.live_spikes_connection.add_start_resume_callback("spike_sender", self.start)

    def start(self, a, b):
        print("Spikeinjection ready\n\n\n")

    def event_callback(self, data, topic):
        with self.lock:
            if self.inject_spikes:
                for event in data.events:
                    if x_left_edge < event.x < x_right_edge and y_top_edge < event.y < y_bottom_edge:
                        x = event.x - x_left_edge
                        y = event.y - y_top_edge
                        n_id = (y * new_res) + x
                        if event.polarity:
                            n_id += new_res*new_res
                        self.live_spikes_connection.send_spike("spike_sender", n_id)


def run_spike_injection(req):
    spike_injector.inject_spikes = True
    return DoSpikeInjectionResponse(True)


def stop_spike_injection(req):
    spike_injector.inject_spikes = False
    return DoStopSpikeInjectionResponse(True)


def get_grouped_n_id(xaddr, yaddr):
    xaddr = np.array(xaddr) // group_by
    yaddr = np.array(yaddr) // group_by
    return (yaddr * (dvs_res / group_by)) + xaddr


spike_injector = SpikeInjector()
dvs_res = 128
new_res = 32
group_by = 1  # 128 // new_res
x_left_edge = dvs_res // 2 - new_res // 2
x_right_edge = dvs_res // 2 + new_res // 2
y_top_edge = dvs_res // 2 - new_res // 2
y_bottom_edge = dvs_res // 2 + new_res // 2


def main(argv=None):
    rospy.init_node("spike_injector")
    s1 = rospy.Service('run_spike_injection', DoSpikeInjection, run_spike_injection)
    s2 = rospy.Service('stop_spike_injection', DoStopSpikeInjection, stop_spike_injection)
    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
