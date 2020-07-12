#!/usr/bin/env python
import os, sys
import experimentLib as elib
import experimentTools as et
import traceback
import pdb
from erbp_utils.erbp_plotter import Plotter
import erbp_utils.file_io as fio
import erbp_utils.rosbag_to_ras as gras
import rospy
import rospkg
from sacc_rec_grasp.srv import *
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2


# "-m yappi" to profile

def run_prediction(req):
    try:
        os.system('rm -rf {path_to_pkg}/scripts/outputs/{directory}/predict'.format(**context))
        os.system('mkdir -p {path_to_pkg}/scripts/outputs/{directory}/predict'.format(**context))
        context['sample_duration_prediction'] = gras.create_single_rosbag_ras(context['directory'],
                                                                              context['path_to_pkg'],
                                                                              req.path_to_rosbag)
        # plotter.plot_2d_input_ras('{}/predict'.format(context['directory']), 32, save=True)
        ret = os.system('mpirun -n {ncores} {path_to_network_topology_executable} \
                --learn false \
                --eta 0.\
                --simtime {sample_duration_prediction} \
                --record_full true \
                --record_rasters true \
                --record_rates true \
                --dir  {path_to_pkg}/scripts/outputs/{directory}/predict/ \
                --fvh  {path_to_pkg}/scripts/inputs/{directory}/train/{fvh} \
                --fho  {path_to_pkg}/scripts/inputs/{directory}/train/{fho} \
                --fhh  {path_to_pkg}/scripts/inputs/{directory}/train/{fhh} \
                --foe  {path_to_pkg}/scripts/inputs/{directory}/train/{foe} \
                --feo  {path_to_pkg}/scripts/inputs/{directory}/train/{feo} \
                --fve  {path_to_pkg}/scripts/inputs/{directory}/train/{fve} \
                --feh  {path_to_pkg}/scripts/inputs/{directory}/train/{feh} \
                --ip_v {path_to_pkg}/scripts/inputs/{directory}/predict/{ip_v}\
                --prob_syn {prob_syn}\
                --nvis {nv} \
                --nhid {nh} \
                --nout {nc} \
                --sigma {sigma}\
                '.format(**context))
        if ret == 0:
            print('ran')

        plotter.plot_ras_spikes(
            '{}/scripts/outputs/{}/predict/coba.*.{}.ras'.format(context['path_to_pkg'], context['directory'], '{}'),
            start=0,
            end=context['sample_duration_prediction'],
            layers=['vis', 'hid'],
            res=context['nv'] - context['nc'],
            number_of_classes=context['nc'],
            save=True,
            input_att_window=False,
            output_path='{}/scripts/html/plots'.format(context['path_to_pkg']),
            plot_label=False)
        plotter.plot_output_spikes_aggregated(
            '{}/scripts/outputs/{}/predict/coba.*.out.ras'.format(context['path_to_pkg'], context['directory']),
            start=0,
            end=context['sample_duration_prediction'],
            classes=['ball', 'bottle', 'nothing', 'pen'],
            save=True,
            output_path='{}/scripts/html/plots'.format(context['path_to_pkg']))
        return DoPredictionResponse(elib.process_prediction(context))
    except Exception as exc:
        rospy.logerr(exc)
        return DoPredictionResponse(-1)


def run_testing(labels_test, sample_duration_test):
    # Uses outputs as inputs for the matrix! This is because the weights are symmetrized and written in the output.
    os.system('rm -rf {path_to_pkg}/scripts/outputs/{directory}/test'.format(**context))
    os.system('mkdir -p {path_to_pkg}/scripts/outputs/{directory}/test'.format(**context))
    ret = os.system('mpirun -n {ncores} {path_to_network_topology_executable} \
        --learn false \
        --eta 0.\
        --simtime {simtime_test} \
        --record_full false \
        --record_rasters false \
        --record_rates true \
        --dir {path_to_pkg}/scripts/outputs/{directory}/test/ \
        --fvh {path_to_pkg}/scripts/inputs/{directory}/train/{fvh} \
        --fho {path_to_pkg}/scripts/inputs/{directory}/train/{fho} \
        --fhh {path_to_pkg}/scripts/inputs/{directory}/train/{fhh} \
        --foe {path_to_pkg}/scripts/inputs/{directory}/train/{foe} \
        --feo {path_to_pkg}/scripts/inputs/{directory}/train/{feo} \
        --fve {path_to_pkg}/scripts/inputs/{directory}/train/{fve} \
        --feh {path_to_pkg}/scripts/inputs/{directory}/train/{feh} \
        --ip_v {path_to_pkg}/scripts/inputs/{directory}/test/{ip_v}\
        --prob_syn {prob_syn}\
        --nvis {nv} \
        --nhid {nh} \
        --nout {nc} \
        --sigma {sigma}\
        '.format(**context))

    if ret == 0:
        print('ran')
    plotter.plot_ras_spikes(
        '{}/scripts/outputs/{}/test/coba.*.{}.ras'.format(context['path_to_pkg'], context['directory'], '{}'), start=0,
        end=4,
        layers=['out'], res=context['nv'] - context['nc'], number_of_classes=context['nc'],
        save=True, input_att_window=False)
    # first 5 labels: 7,2,1,0,4
    rate_class, first_class, rate_confusion_data_frame, first_confusion_data_frame, output_spikes_per_label, ouput_spikes_per_label_norm, snr, snr_per_label = elib.process_test_classification(
        context, sample_duration_test, labels_test)

    plotter.plot_output_spike_count(output_spikes_per_label, 'Output spike count per label', 0, save=True,
                                    image_title='out_spk')
    plotter.plot_output_spike_count(ouput_spikes_per_label_norm, 'Normalized output spike count per label', 0,
                                    save=True,
                                    image_title='out_spk_norm')
    plotter.plot_confusion_matrix(rate_confusion_data_frame, save=True)
    # plotter.plot_confusion_matrix(first_confusion_data_frame, save=True)
    # print('snr per label: {}'.format(snr_per_label))
    # print('total snr: {}'.format(snr))
    return (rate_class, first_class), (snr_per_label, snr)


def run_learn():
    print("eta: " + str(context['eta']))
    os.system('rm -rf {path_to_pkg}/scripts/outputs/{directory}/train'.format(**context))
    os.system('mkdir -p {path_to_pkg}/scripts/outputs/{directory}/train'.format(
        **context))  # /home/alexander/.sources/erbp_auryn/build/release/experiments/exp_rbp_flash
    run_cmd = 'mpirun -n {ncores} {path_to_network_topology_executable} \
        --learn true \
        --simtime {simtime_train} \
        --record_full true \
        --record_rasters true \
        --record_rates true \
        --dir {path_to_pkg}/scripts/outputs/{directory}/train \
        --eta  {eta}\
        --prob_syn {prob_syn}\
        --fvh {path_to_pkg}/scripts/inputs/{directory}/train/{fvh} \
        --fho {path_to_pkg}/scripts/inputs/{directory}/train/{fho} \
        --fhh {path_to_pkg}/scripts/inputs/{directory}/train/{fhh} \
        --foe {path_to_pkg}/scripts/inputs/{directory}/train/{foe} \
        --feo {path_to_pkg}/scripts/inputs/{directory}/train/{feo} \
        --fve {path_to_pkg}/scripts/inputs/{directory}/train/{fve} \
        --feh {path_to_pkg}/scripts/inputs/{directory}/train/{feh} \
        --ip_v {path_to_pkg}/scripts/inputs/{directory}/train/{ip_v}\
        --gate_low  {gate_low}\
        --gate_high  {gate_high}\
        --sigma {sigma}\
        --nvis {nv} \
        --nhid {nh} \
        --nout {nc} \
        '.format(**context)
    ret = os.system(run_cmd)
    return ret, run_cmd


context = {'ncores': 4,
           'directory': 'grasp',
           'nv': (32 * 32) * 2 + 4,  # Include nc
           'nh': 400,
           'nh2': 200,
           'nh1': 200,
           'nc': 4,
           'eta': 6e-04,
           'eta_decay': 0.9,
           'ncpl': 1,
           'gate_low': -.6,
           'gate_high': .6,
           'fvh': 'fwmat_vh.mtx',
           'fho': 'fwmat_ho.mtx',
           'fhh': 'fwmat_hh.mtx',
           'foe': 'fwmat_oe.mtx',
           'feo': 'fwmat_eo.mtx',
           'fve': 'fwmat_ve.mtx',
           'feh': 'fwmat_eh.mtx',
           'ip_v': 'input.ras',  # Hard-coded
           'beta_prm': 1.0,
           'tau_rec': 4e-3,
           'tau_ref': 4e-3,
           'min_p': 1e-5,
           'max_p': .98,
           'binary': False,
           'sigma': 0e-3,
           'n_epochs': 30,
           'n_loop': 1,
           'prob_syn': 0.65,
           'init_mean_bias_v': -.1,
           'init_mean_bias_h': -.1,
           'init_std_bias_v': 1e-32,
           'init_std_bias_h': 1e-32,
           'input_thr': .43,
           'input_scale': .5,
           'mean_weight': 0.0,  # useless
           'std_weight': 7.,
           'test_every': 5,
           'recurrent': False}

rospack = rospkg.RosPack()
context['path_to_pkg'] = rospack.get_path('sacc_rec_grasp')
context['eta_orig'] = context['eta']
plotter = Plotter(context['path_to_pkg'])
image_pub = rospy.Publisher("/agg_output_spike_img", CompressedImage, queue_size=1)


def update_weight_stats(weight_stats):
    stat_dict = (
        fio.get_weight_stats(
            '{}/scripts/inputs/{}/train/fwmat_{}.mtx'.format(context['path_to_pkg'], context['directory'], '{}'),
            context))
    if len(weight_stats) > 0:
        for key in stat_dict.keys():
            weight_stats[key] += stat_dict[key]
    else:
        weight_stats = stat_dict
    return weight_stats


def update_output_weights(output_weights):
    output_mtx = fio.mtx_file_to_matrix('{path_to_pkg}/scripts/inputs/{directory}/train/fwmat_ho.mtx'.format(**context))
    output_mtx = output_mtx[200:, :].flatten()
    output_weights.append(output_mtx)
    return output_weights


def run_load_weights(req):
    folder = req.results_dir
    directory = '{}/scripts/Results/{}/'.format(context['path_to_pkg'], folder)
    if directory is not None:
        print 'Loading previous run...'
        et.globaldata.directory = directory
        M = et.load('M.pkl')
        elib.write_allparameters_rbp(M, context)
    return DoLoadWeightsResponse(True)


def run_training(req):
    try:
        last_perf = (0.0, 0.0)
        init = True
        test = True
        save = True

        n_epochs = context['n_epochs']
        test_every = context['test_every']

        os.system('mkdir -p {path_to_pkg}/scripts/plots/'.format(**context))

        if init:
            os.system('rm -rf {path_to_pkg}/scripts/inputs/{directory}/train/'.format(**context))
            os.system('mkdir -p {path_to_pkg}/scripts/inputs/{directory}/train/'.format(**context))
            elib.create_rbp_init(base_filename='{path_to_pkg}/scripts/inputs/{directory}/train/fwmat'.format(**context),
                                 **context)

        os.system('rm -rf {path_to_pkg}/scripts/inputs/{directory}/test/'.format(**context))
        os.system('mkdir -p {path_to_pkg}/scripts/inputs/{directory}/test/'.format(**context))
        sample_duration_test, labels_test = gras.create_batch_ras(context['path_to_pkg'], context['directory'],
                                                                  context['data_dir'], 'test', context['nc'],
                                                                  cache=True)
        context['simtime_test'] = sample_duration_test[-1]

        acc_hist = []
        snr_hist = []
        weight_stats = {}
        output_weights = []
        spkcnt = [None for i in range(n_epochs)]

        if test:
            res, snr = run_testing(labels_test, sample_duration_test)
            acc_hist.append([0, res])
            snr_hist.append([0, snr])

        # plotter.plot_2d_input_ras('{}/{}'.format(context['directory'], 'test'), 32, 0, 3)

        weight_stats = update_weight_stats(weight_stats)
        output_weights = update_output_weights(output_weights)
        for i in xrange(n_epochs):
            print('\nTrain epoch: {}'.format(i))
            sample_duration_train, labels_train = gras.create_batch_ras(context['path_to_pkg'], context['directory'],
                                                                        context['data_dir'], 'train', context['nc'],
                                                                        cache=True)
            context['simtime_train'] = sample_duration_train[-1]
            # print(context['simtime_train'])
            # print('New train data : {}\n{}'.format(labels_train, sample_duration_train))
            ret, run_cmd = run_learn()
            plotter.plot_2d_input_ras('{}/train'.format(context['directory']), 32, end=2, save=True)
            plotter.plot_ras_spikes(
                '{}/scripts/outputs/{}/train/coba.*.{}.ras'.format(context['path_to_pkg'], context['directory'], '{}'),
                start=0,
                end=sample_duration_train[0] - 0.4,
                layers=['vis', 'hid', 'out'],
                res=context['nv'] - context['nc'],
                number_of_classes=context['nc'],
                save=True,
                input_att_window=False)

            context['eta'] = context['eta'] * context['eta_decay']
            spkcnt[i] = elib.get_spike_count('{path_to_pkg}/scripts/outputs/{directory}/train'.format(**context))
            M = elib.process_parameters_rbp_dual(context)

            plotter.plot_weight_matrix(
                '{}/scripts/inputs/{}/train/fwmat_{}.mtx'.format(context['path_to_pkg'], context['directory'], '{}'),
                save=True)
            plotter.plot_weight_histogram(
                '{}/scripts/inputs/{}/train/fwmat_{}.mtx'.format(context['path_to_pkg'], context['directory'], '{}'),
                nh1=context['nh1'], save=True)
            output_weights = update_output_weights(output_weights)
            weight_stats = update_weight_stats(weight_stats)
            if test_every > 0:
                if i % test_every == test_every - 1:
                    print('\nTest epoch: {}'.format(i))
                    res, snr = run_testing(labels_test, sample_duration_test)
                    acc_hist.append([i + 1, res])
                    snr_hist.append([i + 1, snr])
                    print('ACC: {}\nSNR: {}'.format(res, snr))
                    if res > last_perf:
                        last_perf = res
                        bestM = elib.read_allparamters_dual(context)

        plotter.plot_weight_stats(weight_stats, save=True)
        plotter.plot_output_weights_over_time(output_weights, save=True)
        if len(acc_hist) > 0:
            plotter.plot_accuracy_rate_first(acc_hist, save=True)
            print('ACC hist: {}'.format(acc_hist))
        if len(snr_hist) > 0:
            print('SNR hist: {}'.format(snr_hist))

        if save:
            M = elib.read_allparamters_dual(context)
            results_dir = et.mksavedir(pre='{}/scripts/Results/'.format(context['path_to_pkg']))
            et.globaldata.context = context
            et.save()
            et.save(context, 'context.pkl')
            et.save(sys.argv, 'sysargv.pkl')
            et.save(M, 'M.pkl')
            et.save(spkcnt, 'spkcnt.pkl')
            et.save(acc_hist, 'acc_hist.pkl')
            et.save(snr_hist, 'snr_hist.pkl')
            et.annotate('res', text=str(acc_hist))
            et.save(bestM, 'bestM.pkl')

            elib.textannotate('last_res', text=str(acc_hist))
            elib.textannotate('last_dir', text=results_dir)

            os.system('mv {path_to_pkg}/scripts/plots/ {results_dir}'.format(path_to_pkg=context['path_to_pkg'],
                                                                             results_dir=results_dir))
        return DoTrainingResponse(str(acc_hist))
    except:
        type, value, tb = sys.exc_info()
        traceback.print_exc()
        pdb.post_mortem(tb)


def main():
    rospy.init_node("erbp_ssn_predict")
    context['data_dir'] = rospy.get_param('~data_directory')
    context['path_to_network_topology_executable'] = rospy.get_param('~net_topo_exe',
                                                                     '{path_to_pkg}/network_topology_executable/exp_rbp_flash')
    s1 = rospy.Service('predict', DoPrediction, run_prediction)
    s2 = rospy.Service('train', DoTraining, run_training)
    s3 = rospy.Service('load_weights', DoLoadWeights, run_load_weights)
    rospy.spin()


if __name__ == '__main__':
    exit(main())
