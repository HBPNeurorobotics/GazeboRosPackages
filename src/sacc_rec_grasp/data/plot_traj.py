#!/usr/bin/env python

try:
    import rosbag
except ImportError as e:
    print(e)

import numpy as np
import matplotlib.pyplot as plt

def read_joint_states_from_bag(filename):
    bag = rosbag.Bag(filename)
    joint_states = np.array(list(bag.read_messages(topics=['/arm/joint_states'])))[:, 1]
    bag.close()
    return joint_states

def get_joint_trajectory(joint_states):
    initial_time = joint_states[0].header.stamp.to_sec()
    times = np.array(map(lambda jt: jt.header.stamp.to_sec() - initial_time, joint_states))
    positions = np.array(map(lambda jt: list(jt.position), joint_states))
    joint_names = joint_states[0].name
    return times, positions, joint_names

traj_names = ['ball', 'pen', 'bottle']

for name in traj_names:
    # joint_states = read_joint_states_from_bag(name+'.bag')
    # times, positions, joint_names = get_joint_trajectory(joint_states)
    # np.save(name+'.npy', [times, positions, joint_names])
    times, positions, joint_names = np.load(name+'.npy')

    fig, ax = plt.subplots()
    for n, traj in zip(joint_names, positions.transpose()):
        ax.plot(times, traj, label=n)
    ax.set_xlim([0, 12])
    ax.set_ylim([-2, 2])
    ax.legend(loc='lower right')
    fig.savefig(name+'.png')
