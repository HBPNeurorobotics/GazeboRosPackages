# Micro-Saccade Grasping Demo

This catkin package is the central manager to run the HBP microsaccade grasping demo.
This demo involves the DVS head performing microsaccade, recognizing the object and the schunk arm performing the reach and grasp.
Recognition of the object is achieved with a spiking neural network implemented with Auryn or SpiNNaker.

Install
-----------

Get the **Erbp-Auryn Simulator** here: https://ids-git.fzi.de/hbp/erbp and follow its README instructions.

For the ROS side, if you rely on robot folders, you can use the provided [environment configuration](https://ids-git.fzi.de/core/robot_folders_environments/blob/master/hbp/visuomotor_learning.yaml).

Launch
--------
With the following demo script you start all relevant nodes:
```bash
fzirob run start_demo.sh
fzirob run start_demo_sim.sh
fzirob run start_demo_spinnaker.sh
```

Demo with trained network
--------

If the network is already trained, simply call the service:
```bash
rosservice call /sacc_rec_grasp
```
This will do a saccade -> recognition -> grasp.
You can call this service in a bash loop:
```bash
for i in {1..10}; do;
rosservice call /sacc_rec_grasp
sleep 1
done
```
To monitor the DVS and classification results, connect a web browser to the following URL: http://ids-shuttle-2.fzi.de:8741

Re-training the network
--------

### Recording new data

The data is recorded as rosbags in [the scripts/data/data](scripts/data/data/) folder.

It is important that you move the *right* DVS so that the objects are always centered in middle 32x32 pixels in the frame.
Simply specify the 3D pose relative to the table corner in [manager.launch](launch/manager.launch).

There is a cropper node to help you see what the network see: `dvs_eye_movement/scripts/event_cropper.py`.
Use it with the provided launch file:
```bash
roslaunch sacc_rec_grasp view_rosbag.launch
rosbag play -l pen/1572863202.bag
```

To record a sample, call the following service:
```bash
rosservice call /sacc_record_label "label: b'"
```
Possible labels: 'b' (ball), 'c' (bottle), 'p' (pen), 'n' (nothing).
The total number of samples you should collect (equally distributed across all classes) is specified in [preprocessing_config.yaml](config/preprocessing_config.yaml).

### Retraining the network

Before training the network with new data, remember to clean the cached data:
```bash
rm sacc_rec_grasp/scripts/data/data/*.h5
```

Once you collected your data, you can train the network with the following service call:

_(Auryn)_ Train network:
```
rosservice call /train
```

You can also load previous trained weights with:
_(Auryn)_ Load weights of an old experiment, giving the name of the directory within /Results:
```
rosservice call /load_weights "results_dir: 'foobar'"
```

You can test the network by predicting the label of a rosbag:
```
rosservice call /predict "path_to_rosbag: 'FOO'"
```

Using SpiNNaker
-------------

There are two ways SpiNNaker is integrated in the demo:
- **(preferred)** train a network on SpiNNaker from a dataset and perform inference with live spike injection
- **(unstable)** learn and infer with live spike injection

## SpiNNaker learning on a dataset

The important file is [run_offline_classification_spinnaker.py](scripts/run_offline_classification_spinnaker.py).
It is launched by default in [spiking.launch](launch/spiking.launch) when `spinnaker=True`.

The network is trained with the same dataset as with Auryn, simply call:
```
rosservice call /spinnaker/train
```
After training, the weights are frozen for inference from live spike injection, and saved for future execution.

## (unstable) SpiNNaker learning from live spike injection

The important file is [run_live_classification_spinnaker.py](scripts/run_live_classification_spinnaker.py).
It is launched in [spiking.launch](launch/spiking.launch) when `spinnaker=True` and `learn_online=True`.

_(SpiNNaker)_ Learn online with labels 'b' (ball), 'c' (bottle), 'p' (pen), 'n' (nothing):
```
rosservice call /sacc_learn_online "label: 'b'"
```
_(SpiNNaker)_ Start and stop online classification, returning the result on stop:
```
rosservice call /spinnaker/start_classify
rosservice call /spinnaker/stop_classify
```
_(SpiNNaker)_ Start and stop online spike injection from the DVS:
```
rosservice call /spinnaker/run_spike_injection
rosservice call /spinnaker/stop_spike_injection
```
_(SpiNNaker)_ Change label spike rates:
```
rosservice call /spinnaker/change_label_spike_rate "rates: - FOO neuron_ids: - FOO"
```


Most important files
--------
- **Main Service Provider and manager**: scripts/grasping_manager.py

- **Run Auryn network**: scripts/run_classification.py

- **Run SpiNNaker network**: scripts/run_live_classification_spinnaker.py

- **Live classification of SpiNNaker output**: live_output_spike_receiver.py

- **Auryn Network topology**: network_topology_executable/exp_rbp_flash

- **Evaluate experiment**: scripts/experimentLib.py

- **Plotting**: erbp_utils/erbp_plotter.py

- **Network topology**: network_topology_executable/exp_rbp_flash


License & Copyright (Saccade Recognition Grasp)
---------------------------

Copyright 2017-2019 FZI

Saccade Recognition Grasp uses eRBP DVS built on Auryn and is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Saccade Recognition Grasp is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Auryn.  If not, see <http://www.gnu.org/licenses/>.
