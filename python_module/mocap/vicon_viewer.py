#!/usr/bin/python

# Rafi Hayne

from MocapCommon import *
from MarkerMapper import *
import MocapDrawer
import numpy as np
import os
import rospy
from std_msgs.msg import *
from lightweight_vicon_bridge.msg import *
import subprocess
import Queue
import threading
from openravepy import *


class Tracker:

    def __init__(self, marker_topic, object_topic):
        # self.bag = subprocess.Popen('rosbag play /home/rafi/logging_three/first/2014-07-31-15-50-15.bag', stdin=subprocess.PIPE, stdout=open(os.devnull, 'w'), shell=True, cwd='./')
        # self.bag = subprocess.Popen('rosbag play /home/rafi/logging_fourth/0/2014-08-05-17-58-43.bag', stdin=subprocess.PIPE, stdout=open(os.devnull, 'w'), shell=True, cwd='./')

        # lOGGING 5
        # 1
        # self.bag = subprocess.Popen('rosbag play /home/rafi/logging_five/1/2014-08-06-12-19-59.bag', stdin=subprocess.PIPE, stdout=open(os.devnull, 'w'), shell=True, cwd='./')
        # 2
        # self.bag = subprocess.Popen('rosbag play /home/rafi/logging_five/2/2014-08-06-12-21-42.bag', stdin=subprocess.PIPE, stdout=open(os.devnull, 'w'), shell=True, cwd='./')
        # 3
        # self.bag = subprocess.Popen('rosbag play /home/rafi/logging_five/3/2014-08-06-12-22-42.bag', stdin=subprocess.PIPE, stdout=open(os.devnull, 'w'), shell=True, cwd='./')
        # 4
        self.bag = subprocess.Popen('rosbag play /home/rafi/logging_five/4/2014-08-06-12-24-52.bag', stdin=subprocess.PIPE, stdout=open(os.devnull, 'w'), shell=True, cwd='./')


        self.frames = []
        self.last_frame = None
        self.lock = threading.Lock()

        self.object_q = Queue.Queue()
        self.marker_q = Queue.Queue()

        self.viewer = MocapDrawer.Drawer(NB_MARKERS, NB_HUMAN, ELBOW_PADS, RARM_ONLY)

        if not (marker_topic and object_topic):
            print "At least one topic needed to subscribe to"
            exit(0)
        if marker_topic:
            self.marker_sub = rospy.Subscriber(marker_topic, MocapMarkerArray, self.marker_cb)
        if object_topic:
            self.object_sub = rospy.Subscriber(object_topic, MocapState, self.object_cb)

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

        for m in m_msg.markers:
            marker_list.append( Marker( m.index, m.position.x, m.position.y, m.position.z ) )

        for o in o_msg.tracked_objects:
            segment = o.segments[0]

            temp = Object( segment.name, int(segment.occluded), segment.transform.translation.x, segment.transform.translation.y, segment.transform.translation.z,
                        segment.transform.rotation.x, segment.transform.rotation.y, segment.transform.rotation.z, segment.transform.rotation.w)

            object_list.append(temp)


        frame = Frame( sec, nsec, nb_markers, marker_list, object_list )
        frame.reorder_objects(NB_HUMAN, ELBOW_PADS, RARM_ONLY)


        # TODO check for occluded object
        if self.last_frame is None:
            # Couldn't initialize a marker map. Just draw the raw points
            self.viewer.clear()
            self.try_init(frame)
            self.viewer.draw_frame_raw(frame)
        else:

            # Fix the markers
            frame = frame.get_new_config_by_distance(self.last_frame, THRESHOLD)
            self.last_frame = frame

            # Draw the skeleton
            self.viewer.clear()
            self.viewer.draw_frame_skeleton(frame)
            self.viewer.draw_frame_raw(frame)
            # self.viewer.draw_frame_axes(frame)


        # self.frames.append(frame)

    def try_init(self, frame):
        if frame.count == ( NB_HUMAN * NB_MARKERS):
            pelv_frames = []
            for object in frame.object_list:
                if object and 'Pelvis' in object.id and not object.is_occluded():
                    pelv_frames.append(object.get_rot_matrix())

            if len(pelv_frames) is not NB_HUMAN:
                return

            # Get the marker map for each human
            print "Getting marker name map"
            maps = []

            for pelvis in pelv_frames:
                points = frame.get_n_closest_markers(pelvis, NB_MARKERS)
                maps.append(MarkerMapper(points, pelvis).assign_marker_names(ELBOW_PADS, RARM_ONLY))

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
                frame.set_marker_names(NB_HUMAN, NB_MARKERS, ELBOW_PADS, RARM_ONLY)
                frame.reorder_ids()
                self.last_frame = frame


    def on_shutdown(self):
        print "killing bag process"
        subprocess.Popen('kill ' + str(self.bag.pid), shell=True)
        # self.save_file()
        print "done"



if __name__ == '__main__':

    THRESHOLD   = 0.0025
    # NB_MARKERS  = 18
    NB_HUMAN    = 2
    ELBOW_PADS  = True
    RARM_ONLY   = True
    NB_MARKERS = get_nb_markers(ELBOW_PADS, RARM_ONLY)


    rospy.init_node('vicon_tracker')
    marker_topic = rospy.get_param("~markers_topic", "mocap_markers")
    object_topic = rospy.get_param("~objects_topic", "mocap_tracking" )

    t = Tracker(marker_topic, object_topic)

