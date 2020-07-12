#!/usr/bin/env python
from spynnaker8.external_devices import SpynnakerLiveSpikesConnection
import numpy as np
import rospy
from sacc_rec_grasp.msg import SpikeArray
import sys
from sacc_rec_grasp.srv import DoStartClassifyOnline, DoStartClassifyOnlineResponse, DoStopClassifyOnline, \
    DoStopClassifyOnlineResponse, DoChangePopSpikeRate, DoSetTargetLabel, DoSetTargetLabelResponse
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout


class Classifier:

    def __init__(self):
        self.classes = ['ball', 'bottle', 'nothing', 'pen']
        self.num_output_spikes = np.zeros(len(self.classes))
        self.output_spike_count_pub = rospy.Publisher("output_spike_count", Int32MultiArray)
        self.change_pos_error_spike_rate = rospy.ServiceProxy('/spinnaker/change_pos_error_spike_rate',
                                                              DoChangePopSpikeRate)
        self.change_neg_error_spike_rate = rospy.ServiceProxy('/spinnaker/change_neg_error_spike_rate',
                                                              DoChangePopSpikeRate)
        self.target_label = -1

    def get_predicted_class_index(self):
        return self.num_output_spikes.argmax()


def spike_callback(msg):
    for n_id in msg.neuron_ids:
        classifier.num_output_spikes[n_id] += 1
        if classifier.target_label != -1 and n_id != classifier.target_label:
            classifier.error_pos_live_spikes_connection.send_spike("error_pos_spike_sender", n_id)
    if classifier.target_label != -1 and classifier.target_label not in msg.neuron_ids:
        classifier.error_neg_live_spikes_connection.send_spike("error_neg_spike_sender", classifier.target_label)
    multiArray = Int32MultiArray()
    multiArray.layout = MultiArrayLayout([MultiArrayDimension(size=classifier.num_output_spikes.size)], 0)
    multiArray.data = classifier.num_output_spikes.tolist()
    classifier.output_spike_count_pub.publish(multiArray)
    #
    # if classifier.target_label != -1:
    #     non_target_class_indices = range(len(classifier.classes))
    #     non_target_class_indices.remove(classifier.target_label)
    # classifier.change_pos_error_spike_rate(rates=[50], neuron_ids=[classifier.target_label])
    # classifier.change_neg_error_spike_rate(rates=[50] * len(non_target_class_indices),
    #                                       neuron_ids=non_target_class_indices)


def start_classify(req):
    classifier.num_output_spikes = np.zeros(len(classifier.classes), dtype=int)
    return DoStartClassifyOnlineResponse(True)


def stop_classify(req):
    predicted_class_index = classifier.get_predicted_class_index()
    return DoStopClassifyOnlineResponse(predicted_class_index=predicted_class_index)


def set_target_label(req):
    print("set_target_label to: {}".format(req.target_label))
    classifier.target_label = req.target_label
    return DoSetTargetLabelResponse(True)


classifier = Classifier()


def main(argv=None):
    rospy.init_node("output_spike_receiver")
    classifier.inject_error = rospy.get_param('~inject_error')
    if classifier.inject_error:
        classifier.error_pos_live_spikes_connection = SpynnakerLiveSpikesConnection(receive_labels=None, local_port=19997,
                                                                              send_labels=["error_pos_spike_sender"])
        classifier.error_neg_live_spikes_connection = SpynnakerLiveSpikesConnection(receive_labels=None, local_port=19999,
                                                                              send_labels=["error_neg_spike_sender"])
    s1 = rospy.Service('start_classify', DoStartClassifyOnline, start_classify)
    s2 = rospy.Service('stop_classify', DoStopClassifyOnline, stop_classify)
    if classifier.inject_error:
        s3 = rospy.Service('set_target_label', DoSetTargetLabel, set_target_label)
    rospy.Subscriber('/output_spikes', SpikeArray, spike_callback)
    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
