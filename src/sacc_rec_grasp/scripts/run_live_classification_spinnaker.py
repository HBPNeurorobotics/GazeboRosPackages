#!/usr/bin/env python
import spynnaker8 as Frontend
import spynnaker8.external_devices as ExternalDevices
from pyNN.utility.plotting import Figure, Panel
import matplotlib.pyplot as plt
import rospy
import sys
from sacc_rec_grasp.srv import DoChangePopSpikeRate, DoChangePopSpikeRateResponse
import rospkg
import numpy as np
from sacc_rec_grasp.msg import SpikeArray


class SpikeReceiver(object):

    def __init__(self, nvis, nhid, nc):
        self.nvis = nvis
        self.nhid = nhid
        self.nc = nc

        self.output_spikes_pub = rospy.Publisher('/output_spikes', SpikeArray, queue_size=10)
        self.inject_error = rospy.get_param('/spinnaker/spike_receiver/inject_error')

        timestep = 1.
        Frontend.setup(timestep=timestep, min_delay=1.0, max_delay=144.0)

        number_of_neurons_per_core = 4
        Frontend.set_number_of_neurons_per_core(Frontend.extra_models.IFCurrExpERBP, number_of_neurons_per_core)
        Frontend.set_number_of_neurons_per_core(Frontend.IF_curr_exp, number_of_neurons_per_core)

        neuron_params = {
            "v_thresh": -50.0,  # do not change - hard-coded in C for now
            "v_reset": -70.0,
            "v_rest": -65.0,
            "v": -60.0,
            "i_offset": 0.25}  # DC input - to enable interesting p_j

        input_injector = Frontend.Population(nvis, ExternalDevices.SpikeInjector(),
                                             additional_parameters={'port': 12345}, label="spike_sender")
        self.pop_vis = Frontend.Population(nvis, Frontend.IF_curr_exp(), label="input_pop")
        self.pop_hidden1 = Frontend.Population(nhid, Frontend.extra_models.IFCurrExpERBP(**neuron_params))
        # self.pop_hidden2 = Frontend.Population(nhid, Frontend.extra_models.IFCurrExpERBP(**neuron_params))
        self.pop_out = Frontend.Population(nc, Frontend.extra_models.IFCurrExpERBP(**neuron_params),
                                           label="output_pop")
        self.pop_label = Frontend.Population(nc, Frontend.SpikeSourcePoisson, {'rate': [0, 0, 0, 0]},
                                             label="label_pop")
        if self.inject_error:
            # self.pop_error_pos = Frontend.Population(nc, Frontend.SpikeSourcePoisson, {'rate': [0, 0, 0, 0]},
            #                                          label="err_pop_pos")
            # self.pop_error_neg = Frontend.Population(nc, Frontend.SpikeSourcePoisson, {'rate': [0, 0, 0, 0]},
            #                                          label="err_pop_neg")
            error_pos_injector = Frontend.Population(nc, ExternalDevices.SpikeInjector(),
                                                     additional_parameters={'port': 12346},
                                                     label="error_pos_spike_sender")

            error_neg_injector = Frontend.Population(nc, ExternalDevices.SpikeInjector(),
                                                     additional_parameters={'port': 12347},
                                                     label="error_neg_spike_sender")
            self.pop_error_pos = Frontend.Population(nc, Frontend.IF_curr_exp(), label="err_pop_pos")
            self.pop_error_neg = Frontend.Population(nc, Frontend.IF_curr_exp(), label="err_pop_neg")

            Frontend.Projection(error_pos_injector, self.pop_error_pos, Frontend.OneToOneConnector(),
                                Frontend.StaticSynapse(weight=5.))
            Frontend.Projection(error_neg_injector, self.pop_error_neg, Frontend.OneToOneConnector(),
                                Frontend.StaticSynapse(weight=5.))
        else:
            self.pop_error_pos = Frontend.Population(nc, Frontend.extra_models.ErrorNeuron(tau_m=1000),
                                                     label="err_pop_pos")
            self.pop_error_neg = Frontend.Population(nc, Frontend.extra_models.ErrorNeuron(tau_m=1000),
                                                     label="err_pop_neg")

        tau_err = 200.0
        w_err_to_hid1 = np.random.sample(nc * nhid) * 0.02
        w_err_to_hid2 = np.random.sample(nc * nhid) * 0.02
        w_err_to_out = 0.02
        w_label_to_err = 0.9
        w_out_to_err = 1.0
        np.random.seed(12345)
        factor = 1.0
        w_plastic_vis_to_hid_exc, w_plastic_vis_to_hid_inh = self.zero_smaller_w(np.random.sample(nvis * nhid) * factor,
                                                                                 np.random.sample(nvis * nhid) * factor)
        # w_plastic_hid_to_hid_exc, w_plastic_hid_to_hid_inh = self.zero_smaller_w(np.random.sample(nhid * nhid) * factor,
        #                                                                     np.random.sample(nhid * nhid) * factor)
        w_plastic_hid_to_out_exc, w_plastic_hid_to_out_inh = self.zero_smaller_w(np.random.sample(nhid * nc) * factor,
                                                                                 np.random.sample(nhid * nc) * factor)

        Frontend.Projection(input_injector, self.pop_vis, Frontend.OneToOneConnector(),
                            Frontend.StaticSynapse(weight=5.))

        vis_hid_synapse_plastic_exc = Frontend.Projection(
            self.pop_vis,
            self.pop_hidden1,
            Frontend.AllToAllConnector(),
            synapse_type=self.get_erbp_learning_rule(w_plastic_vis_to_hid_exc, tau_err, timestep),
            receptor_type="excitatory")

        # Create projection from input to hidden neuron using learning rule
        vis_hid_synapse_plastic_inh = Frontend.Projection(
            self.pop_vis,
            self.pop_hidden1,
            Frontend.AllToAllConnector(),
            synapse_type=self.get_erbp_learning_rule(w_plastic_vis_to_hid_inh, tau_err, timestep),
            receptor_type="inhibitory")
        #
        # # Create projection from input to hidden neuron using learning rule
        # hid_hid_synapse_plastic_exc = Frontend.Projection(
        #     self.pop_hidden1,
        #     self.pop_hidden2,
        #     Frontend.AllToAllConnector(),
        #     synapse_type=self.get_erbp_learning_rule(w_plastic_hid_to_hid_exc, tau_err, timestep),
        #     receptor_type="excitatory")
        #
        # # Create projection from input to hidden neuron using learning rule
        # hid_hid_synapse_plastic_inh = Frontend.Projection(
        #     self.pop_hidden1,
        #     self.pop_hidden2,
        #     Frontend.AllToAllConnector(),
        #     synapse_type=self.get_erbp_learning_rule(w_plastic_hid_to_hid_inh, tau_err, timestep),
        #     receptor_type="inhibitory")

        # Create projection from hidden to output neuron using learning rule
        hid_out_synapse_exc = Frontend.Projection(
            self.pop_hidden1,
            self.pop_out,
            Frontend.AllToAllConnector(),
            synapse_type=self.get_erbp_learning_rule(w_plastic_hid_to_out_exc, tau_err, timestep),
            receptor_type="excitatory")

        # Create projection from hidden to output neuron using learning rule
        hid2_out_synapse_inh = Frontend.Projection(
            self.pop_hidden1,
            self.pop_out,
            Frontend.AllToAllConnector(),
            synapse_type=self.get_erbp_learning_rule(w_plastic_hid_to_out_inh, tau_err, timestep),
            receptor_type="inhibitory")

        error_pos_hid1_synapse = Frontend.Projection(
            self.pop_error_pos,
            self.pop_hidden1,
            Frontend.AllToAllConnector(),
            Frontend.StaticSynapse(weight=w_err_to_hid1, delay=timestep),
            receptor_type="exc_err")

        # Create static dendritic projection from error to hidden neuron
        error_neg_hid_synapse = Frontend.Projection(
            self.pop_error_neg,
            self.pop_hidden1,
            Frontend.AllToAllConnector(),
            Frontend.StaticSynapse(weight=w_err_to_hid1, delay=timestep),
            receptor_type="inh_err")

        # Create static dendritic projection from error to hidden neuron
        error_pos_out_synapse = Frontend.Projection(
            self.pop_error_pos,
            self.pop_out,
            Frontend.OneToOneConnector(),
            Frontend.StaticSynapse(weight=w_err_to_out, delay=timestep),
            receptor_type="exc_err")

        # Create static dendritic projection from error to hidden neuron
        error_neg_out_synapse = Frontend.Projection(
            self.pop_error_neg,
            self.pop_out,
            Frontend.OneToOneConnector(),
            Frontend.StaticSynapse(weight=w_err_to_out, delay=timestep),
            receptor_type="inh_err")

        if self.inject_error:
            # ExternalDevices.add_poisson_live_rate_control(self.pop_error_neg, receive_port=20000)
            # ExternalDevices.add_poisson_live_rate_control(self.pop_error_pos)
            # self.poisson_error_neg_control = ExternalDevices.SpynnakerPoissonControlConnection(
            #     poisson_labels=[self.pop_error_neg.label], local_port=20000)
            # self.poisson_error_pos_control = ExternalDevices.SpynnakerPoissonControlConnection(
            #     poisson_labels=[self.pop_error_pos.label])
            ExternalDevices.activate_live_output_for(self.pop_error_pos, database_notify_host="localhost",
                                                     database_notify_port_num=19997)
            ExternalDevices.activate_live_output_for(self.pop_error_neg, database_notify_host="localhost",
                                                     database_notify_port_num=19999)

        else:
            # Create inhibitory static projection from out to error neuron
            out_error_pos_synapse = Frontend.Projection(
                self.pop_out,
                self.pop_error_pos,
                Frontend.OneToOneConnector(),
                # different weight than label_error_synapse because of the non-linear neuron model TODO asap to same weight
                Frontend.StaticSynapse(weight=w_out_to_err, delay=timestep),
                receptor_type="excitatory")

            # Create inhibitory static projection from out to error neuron
            out_error_neg_synapse = Frontend.Projection(
                self.pop_out,
                self.pop_error_neg,
                Frontend.OneToOneConnector(),
                # different weight than label_error_synapse because of the non-linear neuron model TODO asap to same weight
                Frontend.StaticSynapse(weight=w_out_to_err, delay=timestep),
                receptor_type="inhibitory")

            Frontend.Projection(self.pop_label, self.pop_error_pos,
                                Frontend.AllToAllConnector(),
                                Frontend.StaticSynapse(weight=0.05, delay=timestep),
                                receptor_type="label")

            Frontend.Projection(self.pop_label, self.pop_error_neg,
                                Frontend.AllToAllConnector(),
                                Frontend.StaticSynapse(weight=0.05, delay=timestep),
                                receptor_type="label")

            # Create static projection from label to error neuron
            label_error_pos_synapse = Frontend.Projection(
                self.pop_label,
                self.pop_error_pos,
                Frontend.OneToOneConnector(),
                Frontend.StaticSynapse(weight=w_label_to_err, delay=timestep),
                receptor_type="inhibitory")

            # Create static projection from label to error neuron
            label_error_neg_synapse = Frontend.Projection(
                self.pop_label,
                self.pop_error_neg,
                Frontend.OneToOneConnector(),
                Frontend.StaticSynapse(weight=w_label_to_err, delay=timestep),
                receptor_type="excitatory")
            ExternalDevices.add_poisson_live_rate_control(self.pop_label)
            self.poisson_control = ExternalDevices.SpynnakerPoissonControlConnection(
                poisson_labels=[self.pop_label.label])

        ExternalDevices.activate_live_output_for(self.pop_vis, database_notify_host="localhost",
                                                 database_notify_port_num=19996)

        ExternalDevices.activate_live_output_for(self.pop_out, database_notify_host="localhost",
                                                 database_notify_port_num=19998)

        live_spikes_connection_receiver = ExternalDevices.SpynnakerLiveSpikesConnection(receive_labels=["output_pop"],
                                                                                        local_port=19998,
                                                                                        send_labels=None)
        live_spikes_connection_receiver.add_receive_callback("output_pop", self.receive_spikes)

        self.pop_vis.record('spikes')
        self.pop_hidden1.record('spikes')
        # self.pop_hidden2.record('spikes')
        self.pop_out.record('spikes')
        self.pop_label.record('spikes')
        self.pop_error_pos.record('spikes')
        self.pop_error_neg.record('spikes')

    def change_pop_spike_rate(self, pop, label, rates, neuron_ids):
        for neuron_rate in zip(neuron_ids, rates):
            pop.set_rate(label, neuron_rate[0], neuron_rate[1])

    def receive_spikes(self, label, time, neuron_ids):
        self.output_spikes_pub.publish(SpikeArray(neuron_ids, time))

    def zero_smaller_w(self, w_exh, w_inh):
        for i in range(len(w_inh)):
            if w_inh[i] > w_exh[i]:
                w_exh[i] = 0
            else:
                w_inh[i] = 0
        return w_exh, w_inh

    def get_erbp_learning_rule(self, weights, tau_err, timestep):
        return Frontend.STDPMechanism(
            timing_dependence=Frontend.TimingDependenceERBP(
                tau_plus=tau_err, A_plus=1, A_minus=1),
            weight_dependence=Frontend.WeightDependenceERBP(
                w_min=0.0, w_max=1),
            weight=weights,
            delay=timestep)


def change_label_spike_rate(req):
    print("change label spike rate call")
    spike_receiver.change_pop_spike_rate(spike_receiver.poisson_control, spike_receiver.pop_label.label, req.rates,
                                         req.neuron_ids)
    return DoChangePopSpikeRateResponse(True)


def change_pos_error_spike_rate(req):
    print("change pos error spike rate call")
    spike_receiver.change_pop_spike_rate(spike_receiver.poisson_error_pos_control, spike_receiver.pop_error_pos.label,
                                         req.rates, req.neuron_ids)
    return DoChangePopSpikeRateResponse(True)


def change_neg_error_spike_rate(req):
    print("change neg error spike rate call")
    spike_receiver.change_pop_spike_rate(spike_receiver.poisson_error_neg_control, spike_receiver.pop_error_neg.label,
                                         req.rates, req.neuron_ids)
    return DoChangePopSpikeRateResponse(True)


spike_receiver = SpikeReceiver(32 * 32 * 2, 200, 4)


def main(argv=None):
    plot = False

    rospy.init_node("spike_receiver")
    if spike_receiver.inject_error:
        s2 = rospy.Service('change_pos_error_spike_rate', DoChangePopSpikeRate, change_pos_error_spike_rate)
        s3 = rospy.Service('change_neg_error_spike_rate', DoChangePopSpikeRate, change_neg_error_spike_rate)
    else:
        s1 = rospy.Service('change_label_spike_rate', DoChangePopSpikeRate, change_label_spike_rate)
    simtime = 80000
    Frontend.run(simtime)
    # Retrieve spikes from the synfire chain population

    spikes_injected = spike_receiver.pop_vis.get_data('spikes')
    hidden1_spikes = spike_receiver.pop_hidden1.get_data('spikes')
    # hidden2_neuron_data = spike_receiver.pop_hidden2.get_data('spikes')
    spikes_received = spike_receiver.pop_out.get_data('spikes')
    label_spikes = spike_receiver.pop_label.get_data('spikes')
    error_pos_spikes = spike_receiver.pop_error_pos.get_data('spikes')
    error_neg_spikes = spike_receiver.pop_error_neg.get_data('spikes')

    # Clear data structures on spiNNaker to leave the machine in a clean
    # state for future executions

    Frontend.end()

    if plot:
        rospy.loginfo("Making figure")
        Figure(
            # raster plot of the presynaptic neuron spike times
            Panel(spikes_injected.segments[0].spiketrains,
                  yticks=True, markersize=1, xlim=(0, simtime)),
            Panel(hidden1_spikes.segments[0].spiketrains,
                  yticks=True, markersize=2, xlim=(0, simtime)),
            # Panel(hidden2_neuron_data.segments[0].spiketrains,
            #       yticks=True, markersize=2, xlim=(0, simtime)),
            Panel(spikes_received.segments[0].spiketrains,
                  yticks=True, markersize=1, xlim=(0, simtime)),
            Panel(error_pos_spikes.segments[0].spiketrains,
                  yticks=True, markersize=1, xlim=(0, simtime)),
            Panel(error_neg_spikes.segments[0].spiketrains,
                  yticks=True, markersize=1, xlim=(0, simtime)),
            Panel(label_spikes.segments[0].spiketrains,
                  yticks=True, markersize=1, xlim=(0, simtime)),
            title="Simple spike injection", marker=u'|',
            annotations="Simulated with {}".format(Frontend.name())
        )
        rospy.loginfo("Getting plot path")
        rospack = rospkg.RosPack()
        path = rospack.get_path('sacc_rec_grasp')
        rospy.loginfo("Saving figure to {}".format(path))
        plt.savefig('{}/scripts/plots/spiketrain.png'.format(path), dpi=300, bbox_inches='tight')
        rospy.loginfo("Saved figure")

    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
