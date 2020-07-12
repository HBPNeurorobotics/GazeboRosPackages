#!/usr/bin/env python
import sys
import os, errno
import rospy
from sacc_rec_grasp.srv import DoSaccRecGrasp, DoSaccRecGraspResponse, DoSaccRecordLabel, DoSaccRecordLabelResponse, \
    DoSaccLearn, DoSaccLearnResponse, DoPrediction, GraspingState, DoSpikeInjection, DoStopSpikeInjection, \
    DoChangePopSpikeRate, DoStartClassifyOnline, DoStopClassifyOnline, DoSetTargetLabel
from dvs_eye_movements.srv import DoStartRecordRosbag, DoStartRecordRosbagResponse, DoStopRecordRosbag, \
    DoStopRecordRosbagResponse
from fzi_manipulation_msgs.srv import LoadTrajectory, LoadTrajectoryResponse, TrajectoryDesignerSettings
from dvs_head_msgs.srv import DoMicrosaccades, LookAt
from std_msgs.msg import Time
import std_msgs.msg
from geometry_msgs.msg import PointStamped, Point
import rospkg
import time
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from fzi_manipulation_msgs.msg import PlayTrajectoryAction, PlayTrajectoryGoal
from collections import OrderedDict

from visuomotor_msgs.msg import RunNetworkAction, RunNetworkGoal

class Grasping_Manager:
    def __init__(self):
        self.all_classes = OrderedDict(sorted({
                                                  'b': 'ball',
                                                  'c': 'bottle',
                                                  'p': 'pen',
                                                  'n': 'nothing'
                                              }.items(), key=lambda t: t[0]))

        self.do_spike_injection = rospy.ServiceProxy('/spinnaker/run_spike_injection', DoSpikeInjection)
        self.stop_spike_injection = rospy.ServiceProxy('/spinnaker/stop_spike_injection', DoStopSpikeInjection)
        self.change_label_spike_rate = rospy.ServiceProxy('/spinnaker/change_label_spike_rate', DoChangePopSpikeRate)
        self.set_target_label = rospy.ServiceProxy('/spinnaker/set_target_label', DoSetTargetLabel)
        self.do_microsaccade = rospy.ServiceProxy('/head/microsaccade', DoMicrosaccades)
        self.do_grasp = rospy.ServiceProxy('svh/grasping_state', GraspingState)
        self.update_settings = rospy.ServiceProxy('/arm/pathloader/updateSettings', TrajectoryDesignerSettings)
        self.arm_client = actionlib.SimpleActionClient('/arm/arm_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.load_traj = rospy.ServiceProxy('arm/pathloader/loadTrajectory', LoadTrajectory)
        self.do_prediction = rospy.ServiceProxy('predict', DoPrediction)
        self.start_recording = rospy.ServiceProxy('start_record_rosbag', DoStartRecordRosbag)
        self.stop_recording = rospy.ServiceProxy('stop_record_rosbag', DoStopRecordRosbag)
        self.start_classify = rospy.ServiceProxy('spinnaker/start_classify', DoStartClassifyOnline)
        self.stop_classify = rospy.ServiceProxy('spinnaker/stop_classify', DoStopClassifyOnline)
        self.run_network = actionlib.SimpleActionClient('spinnaker/run_network', RunNetworkAction)

        self.look_at = rospy.ServiceProxy('head/look_at', LookAt)

        rospack = rospkg.RosPack()
        path_to_pkg = rospack.get_path('sacc_rec_grasp')
        out_dir = rospy.get_param('~data_directory', 'data')
        eye_reference_point = rospy.get_param('eye_reference_point', [0.9, 0.4, 0.07])
        self.eye_reference_point = Point(*eye_reference_point)

        self.datadir = '{path_to_pkg}/scripts/data/{out_dir}'.format(out_dir=out_dir, path_to_pkg=path_to_pkg)
        self.gen_out_dir()


    def reset_eyes(self):
        rospy.wait_for_service('head/look_at', 5)
        self.look_at(PointStamped(
            header=std_msgs.msg.Header(stamp=rospy.Time.now(),
                          frame_id='table_corner_link'),
            point=self.eye_reference_point
        ))

    def predict_online(self, label=''):
        self.start_classify.wait_for_service(5)
        self.stop_classify.wait_for_service(5)
        self.do_spike_injection.wait_for_service(5)
        self.stop_spike_injection.wait_for_service(5)

        if self.is_learn_online:
            if self.inject_error:
                self.set_target_label.wait_for_service(5)
            else:
                self.change_label_spike_rate.wait_for_service(5)
        self.do_microsaccade.wait_for_service(5)
        self.run_network.wait_for_server(rospy.Duration(5))
        try:
            print("Start spike injection")
            self.start_classify()
            self.run_network.send_goal(RunNetworkGoal(duration=rospy.Duration(2)))

            self.do_spike_injection()
            if label:
                label_neuron_id = self.all_classes.keys().index(label)
                if self.is_learn_online:
                    if self.inject_error:
                        self.set_target_label(label_neuron_id)
                    else:
                        self.change_label_spike_rate(rates=[50], neuron_ids=[label_neuron_id])
            print("Start microsaccade")
            self.do_microsaccade(type=3)
            # time.sleep(1.5)
            print("Stop spike injection")
            self.stop_spike_injection()
            classification_response = self.stop_classify()
            if label and self.is_learn_online:
                if self.inject_error:
                    self.set_target_label(-1)
                else:
                    self.change_label_spike_rate(rates=[0], neuron_ids=[label_neuron_id])
            self.reset_eyes()
            return self.get_predicted_label_by_id(classification_response.predicted_class_index)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def predict(self, path_to_rosbag):
        rospy.wait_for_service('predict')
        try:
            resp1 = self.do_prediction(path_to_rosbag)
            return self.get_predicted_label_by_id(resp1.predicted_label)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def grasp(self, label):
        arm_running = self.arm_client.wait_for_server(rospy.Duration(1))
        if not arm_running:
            rospy.logerr('arm is not running - not performing grasp.')
            return

        try:
            rospy.wait_for_service('svh/grasping_state', 1)
            rospy.wait_for_service('arm/pathloader/updateSettings', 1)
            rospy.wait_for_service('arm/pathloader/loadTrajectory', 1)
        except rospy.ROSException as e:
            rospy.logerr('Motion pipeline not ready.')
            return

        try:
            self.update_settings(rospy.get_param('/arm/velocity', 0.25),
                                 rospy.get_param('/arm/acceleration', 0.50))
        except rospy.service.ServiceException as e:
            rospy.logwarn("updating arm settings exception: {}".format(e))

        traj_per_label = {'ball': ['moveToBallTrajectory', 'dropBallTrajectory', 'moveToStandbyTrajectory'],
                          'bottle': ['moveToBottleTrajectory', 'dropBottleTrajectory', 'moveToStandbyTrajectory'],
                          'pen': ['moveToPenTrajectory', 'dropPenTrajectory', 'moveToStandbyTrajectory']}
        for i, traj in enumerate(traj_per_label[label]):
            load_traj_response = self.load_traj(traj, False)
            if load_traj_response.success:
                if i == 0:
                    self.do_grasp(pose='pre_grasp', label=label)
                arm_client = actionlib.SimpleActionClient('arm/pathloader', PlayTrajectoryAction)
                arm_client.wait_for_server()
                goal = PlayTrajectoryGoal()
                arm_client.send_goal(goal)
                arm_client.wait_for_result()
                if i == 0:
                    self.do_grasp(pose='grasp', label=label)
                    rospy.sleep(1)
                elif i == 1:
                    self.do_grasp(pose='drop', label=label)
                elif i == 2:
                    self.do_grasp(pose='home', label=label)

    def record_label(self, label_char):
        try:
            rospy.wait_for_service('start_record_rosbag', 5)
            rospy.wait_for_service('stop_record_rosbag', 5)
            rospy.wait_for_service('/head/microsaccade', 5)
        except rospy.ROSException as e:
            rospy.logerr("Rosbag recording or microsaccade services not available!")

        for c in self.all_classes.values():
            self.safe_makedirs(os.path.join(self.datadir, c))
        try:
            if label_char == 'predict':
                path = os.path.join(self.datadir, 'prediction.bag')
            else:
                current_class = self.all_classes[label_char]
                path = os.path.join(self.datadir, current_class, str(int(time.time())) + '.bag')
        except KeyError as e:
            print ('No such class: ' + str(e) + '. Possible keys are: {}'.format(self.all_classes.keys()))
        self.start_recording(output_path=path)
        saccade_response = self.do_microsaccade(type=3)
        self.stop_recording()

        if not saccade_response.success:
            rospy.logwarn('Microsaccade failed - reseting eyes to absolute position')
            self.reset_eyes()
        return path

    def gen_out_dir(self):
        self.safe_makedirs(self.datadir)

    def safe_makedirs(self, path):
        try:
            os.makedirs(path)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise

    def get_predicted_label_by_id(self, predicted_label_index):
        predicted_label = self.all_classes.values()[predicted_label_index]
        return predicted_label


def run_sacc_rec_grasp(req):
    if manager.is_predict_online:
        predicted_label = manager.predict_online()
    else:
        path_to_bag = manager.record_label('predict')
        predicted_label = manager.predict(path_to_bag)
        rospy.loginfo("I recognized: {}".format(predicted_label))
    label_pub.publish(std_msgs.msg.String("I recognized: {}".format(predicted_label)))
    if predicted_label in ['ball', 'bottle', 'pen']:
        manager.grasp(predicted_label)
    return DoSaccRecGraspResponse(predicted_label)


def run_sacc_record_label(req):
    manager.record_label(req.label)
    return DoSaccRecordLabelResponse(True, manager.datadir)


def run_sacc_learn(req):
    manager.predict_online(req.label)
    return DoSaccLearnResponse(True)


manager = Grasping_Manager()
label_pub = rospy.Publisher('/status', std_msgs.msg.String, queue_size=10)


def main(argv=None):
    rospy.init_node("grasping_manager")
    manager.is_predict_online = rospy.get_param('~predict_online')
    manager.inject_error = rospy.get_param('~inject_error')
    manager.is_learn_online = rospy.get_param('~learn_online')
    s1 = rospy.Service('sacc_rec_grasp', DoSaccRecGrasp, run_sacc_rec_grasp)
    s2 = rospy.Service('sacc_record_label', DoSaccRecordLabel, run_sacc_record_label)
    if manager.is_learn_online:
        s3 = rospy.Service('sacc_learn_online', DoSaccLearn, run_sacc_learn)
    manager.reset_eyes()

    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
