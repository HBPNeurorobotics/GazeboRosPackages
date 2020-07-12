#!/usr/bin/env python

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt

import spynnaker8 as Frontend
import spynnaker8.external_devices as ExternalDevices
from pyNN.utility.plotting import Figure, Panel

import rospy
import actionlib
import sys
from sacc_rec_grasp.srv import DoChangePopSpikeRate, DoChangePopSpikeRateResponse
import rospkg
import numpy as np
import erbp_utils.rosbag_to_ras as gras
from sacc_rec_grasp.msg import SpikeArray
from sacc_rec_grasp.srv import *
import os
import traceback
import sys
import pdb
import pickle

from visuomotor_msgs.msg import RunNetworkAction, RunNetworkGoal

from collections import Counter

def plot_accuracy(acc, figure_path):
    fig, ax = plt.subplots()
    ax.plot(range(len(acc)), acc)
    ax.set_xlabel("epoch")
    ax.set_ylabel("accuracy")
    plt.savefig(figure_path, dpi=300, bbox_inches='tight')


def plot_weight_hist(weights, figure_path):
    n_hist = len(weights.keys())
    fig, axes = plt.subplots(n_hist, sharex=True, figsize=(6, 2 * n_hist + 1.2))
    for ax, (key, w) in zip(axes, weights.items()):
        ax.hist(w, bins=50)
        ax.text(0.95, 0.95, key,
                transform=ax.transAxes, ha='right', va='top',
                bbox=dict(facecolor='white', alpha=1.0))
    plt.savefig(figure_path, dpi=300, bbox_inches='tight')
    rospy.loginfo("Saved weight histogram figure to {}".format(figure_path))

def plot_population_spikes(spikes, figure_path, t_start=None, duration = 7000):
    if t_start is None:
        import quantities as pq
        t_stop = spikes.values()[0].segments[0].t_stop
         # plot the last seconds of recordings
        xlim = ((t_stop - duration * pq.ms).item(),
                t_stop.item())
    else:
        xlim = (t_start, t_start + duration)

    panels = [
        Panel(pop.segments[0].spiketrains,
              yticks=True, xticks=True, markersize=1, data_labels=[key],
              xlim=xlim, xlabel="Time (ms)")
        for key, pop in spikes.items()
    ]

    Figure(*panels,
           title="Network spikes", marker=u'|',
           annotations="Simulated with {}".format(Frontend.name())
    )

    plt.savefig(figure_path, dpi=300, bbox_inches='tight')
    rospy.loginfo("Saved spiketrain figure to {}".format(figure_path))


def load_data_onto_spinnaker():
    spinnaker_first_run = actionlib.SimpleActionClient('run_network', RunNetworkAction)
    spinnaker_first_run.wait_for_server()
    spinnaker_first_run.send_goal(RunNetworkGoal(duration=rospy.Duration(0.1)))
    spinnaker_first_run.wait_for_result()

class SpinnakerNetwork(object):
    def __init__(self, nvis, nhid, nc, pkg_path):
        self.nvis = nvis
        self.nhid = nhid
        self.nc = nc
        self.pkg_path = pkg_path
        self.plot_every = 1
        self.use_inhibitory_synapse = False

        self.output_spikes_pub = rospy.Publisher('/output_spikes', SpikeArray, queue_size=10)
        self.live_spikes_connection_receiver = None

        self.path_to_weights = rospy.get_param('~weight_path', "{}/weights/spinnaker.pkl".format(pkg_path))
        rospy.loginfo("Save and load spinnaker weights in {}".format(self.path_to_weights))

        self.projections = {}
        self.populations = {}
        self.weights = {}

        self.timestep = 1.

        self.neuron_params_hid = {
            "v_thresh": 30.0,
            "v_reset": 0.0,
            "v_rest": 0.0,
            "i_offset": 0.,
            "v": 0.0,
        }
        self.neuron_params_out = dict(self.neuron_params_hid, **{"i_offset": 1.})

        self.run_as = actionlib.SimpleActionServer('run_network', RunNetworkAction, self.run_network, False)
        self.run_as.start()


    def setup(self):
        if self.live_spikes_connection_receiver is not None and \
           self.live_spikes_connection_receiver.is_connected():
            self.live_spikes_connection_receiver.close()
        Frontend.end()

        Frontend.setup(timestep=self.timestep, min_delay=1.0, max_delay=144.0)
        number_of_neurons_per_core = 4
        Frontend.set_number_of_neurons_per_core(Frontend.extra_models.IFCurrExpERBP, number_of_neurons_per_core)
        # Frontend.set_number_of_neurons_per_core(Frontend.IF_curr_exp, number_of_neurons_per_core)

    def make_inference_network(self, weights=None):
        if weights == None:
            if not os.path.exists(self.path_to_weights):
                rospy.logerr("Tried to instanciate inference network without pre-saved weights!")
                return
            else:
                self.weights = pickle.load(open(self.path_to_weights, 'rb'))

        input_injector = Frontend.Population(self.nvis, ExternalDevices.SpikeInjector(),
                                             additional_parameters={'port': 12345},
                                             label="spike_sender")

        self.populations['pop_vis'] = Frontend.Population(self.nvis, Frontend.IF_curr_exp(), label="input_pop")

        neuron_params_hid = self.neuron_params_hid.copy()
        neuron_params_out = self.neuron_params_out.copy()

        self.populations['pop_hidden1'] = Frontend.Population(self.nhid, Frontend.IF_curr_exp(**neuron_params_hid),
                                                              label="hidden1_pop")
        # self.populations['pop_hidden2'] = Frontend.Population(nhid, Frontend.IF_curr_exp(**neuron_params_hid))
        self.populations['pop_out'] = Frontend.Population(self.nc, Frontend.IF_curr_exp(**neuron_params_out),
                                                          label="output_pop")

        Frontend.Projection(input_injector, self.populations['pop_vis'],
                            Frontend.OneToOneConnector(),
                            Frontend.StaticSynapse(weight=5.))

        self.projections['vis_hid_synapse_plastic_exc'] = Frontend.Projection(
            self.populations['pop_vis'],
            self.populations['pop_hidden1'],
            Frontend.AllToAllConnector(),
            synapse_type=Frontend.StaticSynapse(weight=self.weights['vis_hid_synapse_plastic_exc']),
            receptor_type="excitatory")

        # Create projection from hidden to output neuron using learning rule
        self.projections['hid_out_synapse_plastic_exc'] = Frontend.Projection(
            self.populations['pop_hidden1'],
            self.populations['pop_out'],
            Frontend.AllToAllConnector(),
            synapse_type=Frontend.StaticSynapse(weight=self.weights['hid_out_synapse_plastic_exc']),
            receptor_type="excitatory")

        if self.use_inhibitory_synapse:
            # Create projection from input to hidden neuron using learning rule
            self.projections['vis_hid_synapse_plastic_inh'] = Frontend.Projection(
                self.populations['pop_vis'],
                self.populations['pop_hidden1'],
                Frontend.AllToAllConnector(),
                synapse_type=Frontend.StaticSynapse(weight=self.weights['vis_hid_synapse_plastic_inh']),
                receptor_type="inhibitory")

            # Create projection from hidden to output neuron using learning rule
            self.projections['hid_out_synapse_plastic_inh'] = Frontend.Projection(
                self.populations['pop_hidden1'],
                self.populations['pop_out'],
                Frontend.AllToAllConnector(),
                synapse_type=Frontend.StaticSynapse(weight=self.weights['hid_out_synapse_plastic_inh']),
                receptor_type="inhibitory")



        ExternalDevices.activate_live_output_for(self.populations['pop_vis'], database_notify_host="localhost",
                                                 database_notify_port_num=19996)

        ExternalDevices.activate_live_output_for(self.populations['pop_out'], database_notify_host="localhost",
                                                 database_notify_port_num=19998)
        self.live_spikes_connection_receiver = ExternalDevices.SpynnakerLiveSpikesConnection(receive_labels=["output_pop"],
                                                                                             local_port=19998,
                                                                                             send_labels=None)
        self.live_spikes_connection_receiver.add_receive_callback("output_pop", self.receive_spikes)

        # self.populations['pop_vis'].record('spikes')
        # self.populations['pop_hidden1'].record('spikes')
        # self.populations['pop_hidden2'].record('spikes')
        self.populations['pop_out'].record('spikes')

        # Frontend.run(100) # load the data on spinnaker

    def train_network(self, path_to_bags, n_epochs=5):
        # For convenience, we reuse the auryn workflow with RAS files in the scripts/input folder
        try:
            os.system('mkdir -p {}/scripts/plots/'.format(self.pkg_path))

            os.system('rm -rf {}/scripts/inputs/{}/train/'.format(self.pkg_path, 'grasp'))
            os.system('mkdir -p {}/scripts/inputs/{}/train/'.format(self.pkg_path, 'grasp'))

            os.system('rm -rf {}/scripts/inputs/{}/test/'.format(self.pkg_path, 'grasp'))
            os.system('mkdir -p {}/scripts/inputs/{}/test/'.format(self.pkg_path, 'grasp'))
        except:
            type, value, tb = sys.exc_info()
            traceback.print_exc()
            pdb.post_mortem(tb)

        rospy.loginfo("Creating learning network")

        ret_acc = []
        network_spikes = {}

        for epoch in range(n_epochs):

            epoch_start_time = rospy.Time.now()
            print('\nTrain epoch: {} / {}'.format(epoch, n_epochs))
            sample_duration_train, labels_train = gras.create_batch_ras(self.pkg_path, 'grasp',
                                                                        path_to_bags, 'train',
                                                                        nc=4, cache=True)
            simtime_train = sample_duration_train[-1]

            start_sim_time = int(Frontend.get_current_time())
            current_time = start_sim_time

            input_spike_times, class_spike_times = gras.spike_times_from_ras('{}/scripts/inputs/{}/train/input.ras'.format(self.pkg_path, 'grasp'),
                                                                             self.nvis, self.nc,
                                                                             offset=start_sim_time)
            n_input_spikes = sum([len(train) for train in input_spike_times])
            rospy.loginfo("Current epoch: {} input spikes".format(n_input_spikes))

            self.populations['pop_vis'].set(spike_times=input_spike_times)
            self.populations['pop_label'].set(spike_times=class_spike_times)

            rospy.loginfo("Running training iteration {} for {:3f} seconds".format(epoch, simtime_train))

            # Frontend.run(simtime_train * 1000.) # simulate the whole epoch

            sim_chunk = 25 # chunk of simulation time (in seconds)
            n_chunks = int(simtime_train // sim_chunk)
            for i in range(n_chunks):
                rospy.loginfo("Chunk {} / {}".format(i+1, n_chunks+1))
                Frontend.run(sim_chunk * 1000) # advances simulation by a chunk
            current_time = int(Frontend.get_current_time())
            remaining_time = simtime_train * 1000 + start_sim_time - current_time
            rospy.loginfo("Chunk {n} / {n}\nSimulating last remaining time of {remain:.2f}s".format(n=n_chunks+1,
                                                                                                    remain=remaining_time / 1000.))
            Frontend.run(remaining_time) # advances simulation by a chunk

            wall_time_duration = (rospy.Time.now() - epoch_start_time).to_sec()
            rospy.loginfo("Current epoch finished in {:.2f}s. Real-time factor: {:.2f}%".format(wall_time_duration,
                                                                                                wall_time_duration / simtime_train * 100.))

            for key, pop in self.populations.items():
                network_spikes[key] = pop.get_data('spikes', clear=True) # clear spike buffer
            ret_acc.append(self.compute_accuracy(network_spikes['pop_out'].segments[0].spiketrains,
                                                 start_sim_time,
                                                 start_sim_time + np.array(sample_duration_train) * 1000.,
                                                 labels_train))
            rospy.loginfo("Current epoch accuracy: {:.2f}%".format(ret_acc[-1] * 100.))

            if epoch % self.plot_every == 0:
                figure_path = "{}/scripts/plots/spiketrain_epoch_{}.png".format(self.pkg_path, epoch)
                plot_population_spikes(network_spikes, figure_path, t_start = start_sim_time)
                self.weights = self.retrieve_plastic_weights()
                figure_path = "{}/scripts/plots/weights_epoch_{}.png".format(self.pkg_path, epoch)
                plot_weight_hist(self.weights, figure_path=figure_path)

            # Frontend.run(int(simtime_train * 1000.))

            # ret = {
            #     'spikes_injected': self.populations['pop_vis'].get_data('spikes', clear=True),
            #     'hidden1_spikes': self.populations['pop_hidden1'].get_data('spikes', clear=True),
            #     'spikes_received': self.populations['pop_out'].get_data('spikes', clear=True),
            #     'label_spikes': self.populations['pop_label'].get_data('spikes', clear=True),
            #     'error_pos_spikes': self.populations['pop_error_pos'].get_data('spikes', clear=True),
            #     'error_neg_spikes': self.populations['pop_error_neg'].get_data('spikes', clear=True)
            # }

            # rospy.loginfo("Number of hidden spikes: {}".format(len(ret['hidden1_spikes'].segments[0].spiketrains)))

            # Clear data structures on spiNNaker to leave the machine in a clean state for future executions
            # Frontend.clear()

        # retrieve weights
        rospy.loginfo("Training finished - retrieving weights")
        self.weights = self.retrieve_plastic_weights()
        rospy.loginfo("Saving weights to {}".format(self.path_to_weights))
        pickle.dump(self.weights, open(self.path_to_weights, "wb" ))

        return network_spikes, ret_acc

    def retrieve_plastic_weights(self):
        weights = {}
        for conn in self.projections.keys():
            if '_plastic_' in conn:
                weights[conn] = self.projections[conn].getWeights().tolist()
        return weights

    def compute_accuracy(self, out_spikes, start_sim_time, sample_delimitations, labels):
        sample_begin = start_sim_time
        n_correct_classifications = 0.
        for sample_end, y in zip(sample_delimitations, labels):
            spike_counter = Counter()
            for neuron_idx, spiketrain in enumerate(out_spikes):
                spike_counter[neuron_idx] = len(spiketrain[ (spiketrain >= sample_begin) & (spiketrain < sample_end) ])
            winner_idx = spike_counter.most_common(n=1)[0][0]
            n_correct_classifications += (winner_idx == y)
            sample_begin = sample_end
        return n_correct_classifications / len(labels)

    def make_learning_network(self, input_spike_times=[], class_spike_times=[]):
        self.populations['pop_vis'] = Frontend.Population(self.nvis, Frontend.SpikeSourceArray(spike_times=input_spike_times), label="input_pop")
        self.populations['pop_label'] = Frontend.Population(self.nc, Frontend.SpikeSourceArray(spike_times=class_spike_times), label="label_pop")

        neuron_params_hid = dict(self.neuron_params_hid, **{"tau_err": 1000})
        neuron_params_out = dict(self.neuron_params_out, **{"tau_err": 1000})

        self.populations['pop_hidden1'] = Frontend.Population(self.nhid, Frontend.extra_models.IFCurrExpERBP(**neuron_params_hid),
                                                              label="hidden1_pop")
        # self.populations['pop_hidden2'] = Frontend.Population(nhid, Frontend.extra_models.IFCurrExpERBP(**neuron_params))
        self.populations['pop_out'] = Frontend.Population(self.nc, Frontend.extra_models.IFCurrExpERBP(**neuron_params_out),
                                           label="output_pop")

        w_err_to_hid1 = np.random.sample(self.nc * self.nhid) * 10.
        w_err_to_out = 10.

        w_vis_to_hid = 0.2
        w_hid_to_out = 0.1
        # w_learning_inh = 0.5 * w_learning_exc

        w_label_to_err = 1.0
        w_out_to_err = w_label_to_err

        self.populations['pop_error_pos'] = Frontend.Population(self.nc, Frontend.extra_models.ErrorNeuron(tau_m=1000),
                                                                label="err_pop_pos")
        self.populations['pop_error_neg'] = Frontend.Population(self.nc, Frontend.extra_models.ErrorNeuron(tau_m=1000),
                                                                label="err_pop_neg")

        self.projections['vis_hid_synapse_plastic_exc'] = Frontend.Projection(
            self.populations['pop_vis'],
            self.populations['pop_hidden1'],
            Frontend.AllToAllConnector(),
            synapse_type=self.get_erbp_learning_rule(w_vis_to_hid, reg_rate=1.),
            receptor_type="excitatory")

        # Create projection from hidden to output neuron using learning rule
        self.projections['hid_out_synapse_plastic_exc'] = Frontend.Projection(
            self.populations['pop_hidden1'],
            self.populations['pop_out'],
            Frontend.AllToAllConnector(),
            synapse_type=self.get_erbp_learning_rule(w_hid_to_out),
            receptor_type="excitatory")


        if self.use_inhibitory_synapse:
            # Create projection from input to hidden neuron using learning rule
            self.projections['vis_hid_synapse_plastic_inh'] = Frontend.Projection(
                self.populations['pop_vis'],
                self.populations['pop_hidden1'],
                Frontend.AllToAllConnector(),
                synapse_type=self.get_erbp_learning_rule(w_vis_to_hid),
                receptor_type="inhibitory")

            # Create projection from hidden to output neuron using learning rule
            self.projections['hid_out_synapse_plastic_inh'] = Frontend.Projection(
                self.populations['pop_hidden1'],
                self.populations['pop_out'],
                Frontend.AllToAllConnector(),
                synapse_type=self.get_erbp_learning_rule(w_hid_to_out),
                receptor_type="inhibitory")

        self.projections['error_pos_hid1_synapse'] = Frontend.Projection(
            self.populations['pop_error_pos'],
            self.populations['pop_hidden1'],
            Frontend.AllToAllConnector(),
            Frontend.StaticSynapse(weight=w_err_to_hid1, delay=self.timestep),
            receptor_type="inh_err")

        # Create static dendritic projection from error to hidden neuron
        self.projections['error_neg_hid_synapse'] = Frontend.Projection(
            self.populations['pop_error_neg'],
            self.populations['pop_hidden1'],
            Frontend.AllToAllConnector(),
            Frontend.StaticSynapse(weight=w_err_to_hid1, delay=self.timestep),
            receptor_type="exc_err")

        # Create static dendritic projection from error to hidden neuron
        self.projections['error_pos_out_synapse'] = Frontend.Projection(
            self.populations['pop_error_pos'],
            self.populations['pop_out'],
            Frontend.OneToOneConnector(),
            Frontend.StaticSynapse(weight=w_err_to_out, delay=self.timestep),
            receptor_type="inh_err")

        # Create static dendritic projection from error to hidden neuron
        self.projections['error_neg_out_synapse'] = Frontend.Projection(
            self.populations['pop_error_neg'],
            self.populations['pop_out'],
            Frontend.OneToOneConnector(),
            Frontend.StaticSynapse(weight=w_err_to_out, delay=self.timestep),
            receptor_type="exc_err")

        # Create inhibitory static projection from out to error neuron
        self.projections['out_error_pos_synapse'] = Frontend.Projection(
            self.populations['pop_out'],
            self.populations['pop_error_pos'],
            Frontend.OneToOneConnector(),
            Frontend.StaticSynapse(weight=w_out_to_err, delay=self.timestep),
            receptor_type="excitatory")

        # Create static projection from label to error neuron
        self.projections['label_error_pos_synapse'] = Frontend.Projection(
            self.populations['pop_label'],
            self.populations['pop_error_pos'],
            Frontend.OneToOneConnector(),
            Frontend.StaticSynapse(weight=w_label_to_err, delay=self.timestep),
            receptor_type="inhibitory")

        # Create inhibitory static projection from out to error neuron
        self.projections['out_error_neg_synapse'] = Frontend.Projection(
            self.populations['pop_out'],
            self.populations['pop_error_neg'],
            Frontend.OneToOneConnector(),
            Frontend.StaticSynapse(weight=w_out_to_err, delay=self.timestep),
            receptor_type="inhibitory")

        # Create static projection from label to error neuron
        self.projections['label_error_neg_synapse'] = Frontend.Projection(
            self.populations['pop_label'],
            self.populations['pop_error_neg'],
            Frontend.OneToOneConnector(),
            Frontend.StaticSynapse(weight=w_label_to_err, delay=self.timestep),
            receptor_type="excitatory")

        self.projections['label_error_pos_label_synapse'] = Frontend.Projection(
            self.populations['pop_label'],
            self.populations['pop_error_pos'],
            Frontend.AllToAllConnector(),
            Frontend.StaticSynapse(weight=0.05, delay=self.timestep),
            receptor_type="label")

        self.projections['label_error_neg_label_synapse'] = Frontend.Projection(
            self.populations['pop_label'], self.populations['pop_error_neg'],
            Frontend.AllToAllConnector(),
            Frontend.StaticSynapse(weight=0.05, delay=self.timestep),
            receptor_type="label")

        self.populations['pop_vis'].record('spikes')
        self.populations['pop_hidden1'].record('spikes')
        self.populations['pop_out'].record('spikes')
        self.populations['pop_label'].record('spikes')
        self.populations['pop_error_pos'].record('spikes')
        self.populations['pop_error_neg'].record('spikes')

    def receive_spikes(self, label, time, neuron_ids):
        self.output_spikes_pub.publish(SpikeArray(neuron_ids, time))

    def zero_smaller_w(self, w_exh, w_inh):
        for i in range(len(w_inh)):
            if w_inh[i] > w_exh[i]:
                w_exh[i] = 0
            else:
                w_inh[i] = 0
        return w_exh, w_inh

    def get_erbp_learning_rule(self, init_weight_factor=0.2, tau_err=20., l_rate=1., reg_rate=0.):
        weight_dist = Frontend.RandomDistribution(
            distribution='normal_clipped', mu=init_weight_factor, sigma=init_weight_factor,
            low=0.0, high=2*init_weight_factor)

        return Frontend.STDPMechanism(
            timing_dependence=Frontend.TimingDependenceERBP(
                tau_plus=tau_err, A_plus=l_rate, A_minus=l_rate),
            weight_dependence=Frontend.WeightDependenceERBP(
                w_min=0.0, w_max=1, reg_rate=reg_rate),
            weight=weight_dist,
            delay=self.timestep)

    def run_training(self, req):
        self.setup()
        self.make_learning_network()

        rosbag_datadir = rospy.get_param('~data_directory')
        spikes, acc = self.train_network(rosbag_datadir)

        figure_path = "{}/scripts/plots/spinnaker_accuracy.png".format(self.pkg_path)
        plot_accuracy(acc, figure_path)

        rospy.loginfo("Training finished. Instanciating inference network with fixed weights")
        self.setup()
        self.make_inference_network(self.weights)
        load_data_onto_spinnaker()

        return DoTrainingResponse(str(acc[-1]))

    def run_network(self, goal):
        Frontend.run(goal.duration.to_sec() * 1000)
        rospy.loginfo("Finished running")
        self.run_as.set_succeeded()


def main(argv=None):
    rospy.init_node("offline_spinnaker")
    np.random.seed(12345)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('sacc_rec_grasp')

    spinn_network = SpinnakerNetwork(nvis=32 * 32 * 2,
                                     nhid=200,
                                     nc=4,
                                     pkg_path=pkg_path)
    # make inference network with pre-trained weights
    spinn_network.setup()
    spinn_network.make_inference_network()
    load_data_onto_spinnaker()

    do_training = rospy.Service('train', DoTraining, spinn_network.run_training)

    def shutdown_hook():
        Frontend.end()

    rospy.on_shutdown(shutdown_hook)

    rospy.loginfo("SpiNNaker network ready")

    rospy.spin()

if __name__ == "__main__":
    main()
