#!/usr/bin/python

# Rafi Hayne

from MocapCommon import *
from MarkerMapper import *
import MocapDrawer
from TestBioHumanIk import *

from openravepy import *
import numpy as np
import os
import subprocess
import Queue
import threading

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from lightweight_vicon_bridge.msg import *


class Tracker:

    def __init__(self, nb_humans, elbow_pads,  wrist_pads, rarm_only, nb_markers, marker_topic, object_topic):
        # self.bag = subprocess.Popen('rosbag play /home/rafi/logging_three/first/2014-07-31-15-50-15.bag', stdin=subprocess.PIPE, stdout=open(os.devnull, 'w'), shell=True, cwd='./')
        # self.bag = subprocess.Popen('rosbag play /home/rafi/logging_fourth/0/2014-08-05-17-58-43.bag', stdin=subprocess.PIPE, stdout=open(os.devnull, 'w'), shell=True, cwd='./')

        # self.bag = subprocess.Popen('rosbag play /home/pr2command/workspace/test_mocap/tracker_test/0/2014-12-12-12-25-37.bag', stdin=subprocess.PIPE, stdout=open(os.devnull, 'w'), shell=True, cwd='./')
        # self.bag = subprocess.Popen('rosbag play /home/pr2command/workspace/test_mocap/ricoun_test/0/2014-12-11-14-37-41.bag', stdin=subprocess.PIPE, stdout=open(os.devnull, 'w'), shell=True, cwd='./')

        self.i = 0 # Frame drawn

        self.frames = []
        self.last_frame = None
        self.lock = threading.Lock()

        self.object_q = Queue.Queue()
        self.marker_q = Queue.Queue()

        self.nb_markers = nb_markers

        if rarm_only:
            environment_file = ORMODELS + "/humans_bio_env.xml"
        else:
            environment_file = ORMODELS + "/humans_env_two_arms.xml"

        self.viewer = MocapDrawer.Drawer(nb_markers, nb_humans, elbow_pads, wrist_pads, rarm_only, environment_file)

        self.ik = TestBioHumanIk()
        self.ik.nb_humans           = nb_humans
        self.ik.elbow_pads          = elbow_pads
        self.ik.wrist_pads          = wrist_pads
        self.ik.rarm_only           = rarm_only
        self.ik.environment_file    = environment_file
        self.ik.initialize("", "", self.viewer)
        print "done initializing"

        print "marker_topic : " , marker_topic
        print "object_topic : " , object_topic

        if not (marker_topic and object_topic):
            print "At least one topic needed to subscribe to"
            exit(0)
        if marker_topic:
            self.marker_sub = rospy.Subscriber(marker_topic, MocapMarkerArray, self.marker_cb)
        if object_topic:
            self.object_sub = rospy.Subscriber(object_topic, MocapState, self.object_cb)

        self.joint_state_pub = rospy.Publisher('mocap_human_joint_state', JointState)

        rospy.spin()

    def marker_cb(self, msg):

        self.lock.acquire()
        self.marker_q.put(msg)

        if not self.object_q.empty():
            self.make_frame()

        self.lock.release()

    def object_cb(self, msg):

        self.lock.acquire()
        self.object_q.put(msg)

        if not self.marker_q.empty():
            self.make_frame()

        self.lock.release()

    def make_frame(self):

        marker_list = []
        object_list = []
        m_msg = self.marker_q.get()
        o_msg = self.object_q.get()

        sec = int(m_msg.header.stamp.secs)
        nsec = float(m_msg.header.stamp.nsecs)
        nb_markers = len(m_msg.markers)

        # Get marker list
        marker_list = [Marker( m.index, m.position.x, m.position.y, m.position.z ) for m in m_msg.markers]

        # Object list
        for o in o_msg.tracked_objects:
            segment = o.segments[0]

            temp = Object( segment.name, int(segment.occluded), segment.transform.translation.x, segment.transform.translation.y, segment.transform.translation.z,
                        segment.transform.rotation.x, segment.transform.rotation.y, segment.transform.rotation.z, segment.transform.rotation.w)

            object_list.append(temp)

        frame = Frame( sec, nsec, nb_markers, marker_list, object_list )
        frame.reorder_objects(self.ik.nb_humans, self.ik.elbow_pads, self.ik.wrist_pads, self.ik.rarm_only)

        if self.i % 4 == 0:
            del self.viewer.handles[:]
        self.i += 1

        # TODO check for occluded object
        if self.last_frame is None:
            # Couldn't initialize a marker map. Just draw the raw points
            self.viewer.clear()
            self.try_init(frame)
            del self.ik.handles[:]
            self.viewer.draw_frame_raw(frame)
        else:

            # Fix the markers
            frame = frame.get_new_config_by_distance(self.last_frame, THRESHOLD)
            self.last_frame = frame

            # Draw the skeleton
            del self.ik.handles[:]
            self.viewer.clear()
            #self.viewer.draw_frame_skeleton(frame)
            #self.viewer.draw_frame_raw(frame)
            #self.viewer.draw_frame_axes(frame)
            self.ik.draw_frame(frame, False)

            # Publish joint state to topic
            self.publish_joint_state()

        # self.frames.append(frame)

        self.publish_joint_state()

    def publish_joint_state(self):

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time()

        joints = self.ik.humans[0].GetJoints()

        if len(joints) != self.ik.humans[0].GetDOF():
            rospy.logerror("OpenRave human model is not consistant")
            return

        joint_state.name        = [""]*len(joints)
        joint_state.position    = [0.]*len(joints)

        q_cur = self.ik.humans[0].GetDOFValues()

        for i, joint in enumerate(joints):
            joint_state.name[i] = joint.GetName()
            joint_state.position[i] = q_cur[joint.GetDOFIndex()]

        # Publish joint states
        self.joint_state_pub.publish(joint_state)

    def try_init(self, frame):
        if frame.count == ( NB_HUMAN * NB_MARKERS):
            pelv_frames = []
            for object in frame.object_list:
                if object and 'Pelvis' in object.id and not object.is_occluded():
                    pelv_frames.append(object.get_transform())

            if len(pelv_frames) is not NB_HUMAN:
                return

            # Get the marker map for each human
            print "Getting marker name map"
            maps = []

            for pelvis in pelv_frames:
                points = frame.get_n_closest_markers(pelvis, self.nb_markers)
                maps.append(MarkerMapper(points, pelvis).assign_marker_names(self.ik.elbow_pads, self.ik.wrist_pads, self.ik.rarm_only))

            # Reorder markers according to map
            new_marker_list = []
            for map in maps:
                print map
                for id in map:
                    new_marker_list.append(frame.marker_list[id])

            print "Concatenated marker list"
            frame.marker_list = new_marker_list
            frame.print_marker_ids()

            if frame.get_duplicate_ids() or len(frame.marker_list) != frame.count:
                return

            else:
                print "Found good calibration frame"
                print "Setting marker names"
                frame.set_marker_names(self.ik.nb_humans, self.nb_markers, self.ik.elbow_pads, self.ik.wrist_pads, self.ik.rarm_only)
                frame.reorder_ids()
                self.last_frame = frame


    def on_shutdown(self):
        print "killing bag process"
        subprocess.Popen('kill ' + str(self.bag.pid), shell=True)
        # self.save_file()
        del self.ik
        del self.viewer
        print "done"

if __name__ == '__main__':

    rospy.init_node('mocap_human_skeleton_tracker')

    marker_topic = rospy.get_param("~markers_topic", "mocap_markers")
    object_topic = rospy.get_param("~objects_topic", "mocap_tracking" )

    ORMODELS     = rospy.get_param("~ormodels", "../ormodels" )

    THRESHOLD    = rospy.get_param("~human_tracker_threshold", 0.0025)
    NB_HUMAN     = rospy.get_param("~human_tracker_nb_humans", 1)
    ELBOW_PADS   = rospy.get_param("~human_tracker_elbow_pads", True)
    WRIST_PADS   = rospy.get_param("~human_tracker_wrist_pads", True)
    RARM_ONLY    = rospy.get_param("~human_tracker_rarm_only", True)

    NB_MARKERS   = get_nb_markers(ELBOW_PADS, WRIST_PADS, RARM_ONLY)

    t = Tracker(NB_HUMAN, ELBOW_PADS,  WRIST_PADS, RARM_ONLY, NB_MARKERS, marker_topic, object_topic)
