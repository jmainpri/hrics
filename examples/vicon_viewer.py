#!/usr/bin/python

# Rafi Hayne

import openravepy
from TransformMatrix import *
from get_marker_names import *
import csv
import numpy as np
import os
import collections
import time
import math
import sys
import copy
import rospy
from std_msgs.msg import *
from lightweight_vicon_bridge.msg import *
import subprocess
import Queue
import threading
from openravepy import *

THRESHOLD   = 0.0025
NB_MARKERS  = 18
NB_HUMAN    = 2

class Drawer():

    def __init__(self):

        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.Reset()
        self.env.Load("../ormodels/human_wpi_bio.xml")

        self.human = self.env.GetRobots()[0]
        self.handles = []

    def draw_frame_skeleton(self, frame):
            del self.handles[:]

            humans = []
            humans_raw = []

            for i in range(0, NB_MARKERS*NB_HUMAN, NB_MARKERS):
                Chest = (frame.marker_list[i].array + frame.marker_list[i+1].array)/2
                Sternum = (frame.marker_list[i+2].array + frame.marker_list[i+3].array)/2
                rShoulder = (frame.marker_list[i+4].array + frame.marker_list[i+5].array)/2
                rElbow = (frame.marker_list[i+6].array + frame.marker_list[i+7].array)/2
                rWrist = (frame.marker_list[i+8].array + frame.marker_list[i+9].array)/2
                rPalm = frame.marker_list[i+10].array
                lShoulder = (frame.marker_list[i+11].array + frame.marker_list[i+12].array)/2
                lElbow = (frame.marker_list[i+13].array + frame.marker_list[i+14].array)/2
                lWrist = (frame.marker_list[i+15].array + frame.marker_list[i+16].array)/2
                lPalm = frame.marker_list[i+17].array
                Pelvis = frame.object_list[ (i/18)*2 ].array
                Head = frame.object_list[ (i/18)*2 + 1 ].array

                ChestFront_raw      = frame.marker_list[i].array
                ChestBack_raw       = frame.marker_list[i+1].array
                SternumFront_raw    = frame.marker_list[i+2].array
                SternumBack_raw     = frame.marker_list[i+3].array
                rShoulderFront_raw  = frame.marker_list[i+4].array
                rShoulderBack_raw   = frame.marker_list[i+5].array
                rElbowOuter_raw     = frame.marker_list[i+6].array
                rElbowInner_raw     = frame.marker_list[i+7].array
                rWristOuter_raw     = frame.marker_list[i+8].array
                rWristInner_raw     = frame.marker_list[i+9].array
                rPalm_raw           = frame.marker_list[i+10].array
                lShoulderFront_raw  = frame.marker_list[i+11].array
                lShoulderBack_raw   = frame.marker_list[i+12].array
                lElbowOuter_raw     = frame.marker_list[i+13].array
                lElbowInner_raw     = frame.marker_list[i+14].array
                lWristOuter_raw     = frame.marker_list[i+15].array
                lWristInner_raw     = frame.marker_list[i+16].array
                lPalm_raw           = frame.marker_list[i+17].array

                humans.append( np.array([Chest, Sternum, rShoulder, rElbow, rWrist, rPalm, lShoulder, lElbow, lWrist, lPalm, Pelvis, Head]) )
                humans_raw.append( np.array([ChestFront_raw, ChestBack_raw, SternumFront_raw, SternumBack_raw, rShoulderFront_raw, rShoulderBack_raw,
                                            rElbowOuter_raw, rElbowInner_raw, rWristOuter_raw, rWristInner_raw, rPalm_raw, lShoulderFront_raw, 
                                            lShoulderBack_raw, lElbowOuter_raw, lElbowInner_raw, lWristOuter_raw, lWristInner_raw, lPalm_raw]) )

            for human in humans:
                self.draw_skeleton(human)

            for human in humans_raw:
                self.draw_points(human)

            


    def draw_frame_raw(self, frame):
        del self.handles[:]

        point_list = []

        for m in frame.marker_list:
            point_list.append(m.array)
        for o in frame.object_list:
            point_list.append(o.array)

        self.draw_points(np.array(point_list))


    def draw_skeleton(self, point_list):
        points = point_list

        colors = []
        nb_points = len(points)
        for n in linspace(0.0, 1.0, num=nb_points):
            colors.append((float(n)*1, (1-float(n))*1, 0))

        # Marker points
        self.handles.append(self.env.plot3(points=point_list, pointsize=0.02, colors=array(colors), drawstyle=1))

        # Connect shoulders
        shoulders = np.array([point_list[2],point_list[6]])
        self.handles.append(self.env.drawlinestrip(points=shoulders, linewidth=3.0))

        # Connect head
        head = np.array( [ point_list[1], point_list[11] ] )
        self.handles.append(self.env.drawlinestrip(points=head, linewidth=3.0))

        # Connect right arm
        right_arm = np.array([point_list[2],point_list[3], point_list[4], point_list[5]])
        self.handles.append(self.env.drawlinestrip(points=right_arm, linewidth=3.0))

        # Left Arm
        left_arm = np.array( [point_list[6], point_list[7], point_list[8], point_list[9]] )
        self.handles.append(self.env.drawlinestrip(points=left_arm, linewidth=3.0))

        # Pelvis
        pelv = np.array( [point_list[2], point_list[10], point_list[6]] )
        self.handles.append(self.env.drawlinestrip(points=pelv, linewidth=3.0))

    def draw_points(self, point_list):
        points = point_list

        colors = []
        nb_points = len(points)
        for n in linspace(0.0, 1.0, num=nb_points):
            colors.append((float(n)*1, (1-float(n))*1, 0))

        self.handles.append(self.env.plot3(points=point_list, pointsize=0.02, colors=array(colors), drawstyle=1))

class Marker:
    def __init__(self, id, x, y, z, name=''):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.name = name
        self.array = np.array([self.x, self.y, self.z])
        self.times_dropped = 0

    def is_same_marker(self, a_marker, threshold):
        if self.get_dist(a_marker) > threshold:
            return False
        else:
            return True

    def numpy(self):
        return self.array

    def get_dist(self, other):
        diff = self.array - other.array
        return np.dot(diff, diff.conj())


class Object:
    def __init__(self, id, occluded, x, y, z, r_x, r_y, r_z, r_w):
        self.id = id
        self.occluded = occluded
        self.x = x
        self.y = y
        self.z = z
        self.r_x = r_x
        self.r_y = r_y
        self.r_z = r_z
        self.r_w = r_w
        self.array = np.array([self.x, self.y, self.z])

    def is_occluded(self):
        return self.occluded

    def get_rot_matrix(self):
        mat =  MakeTransform( openravepy.rotationMatrixFromQuat(np.array([self.r_x, self.r_y, self.r_z, self.r_w])), np.transpose(np.matrix([self.x, self.y, self.z])))

        if self.id == 'Pelvis0':
            x_dir = np.array(np.transpose(mat[:,0]).tolist()[0][:3])
            y_dir = np.array(np.transpose(mat[:,1]).tolist()[0][:3])
            z_dir = np.array(np.transpose(mat[:,2]).tolist()[0][:3])

            new_x = -z_dir
            new_x[2] = 0
            new_x = new_x/np.linalg.norm(new_x)
            new_z = np.array([0,0,1])

            new_y = np.cross(new_x, new_z)
            new_y = new_y/np.linalg.norm(new_y)
            new_y = -new_y

            mat[0][0, 0] = new_x[0]
            mat[0][0, 1] = new_y[0]
            mat[0][0, 2] = new_z[0]
            mat[1][0, 0] = new_x[1]
            mat[1][0, 1] = new_y[1]
            mat[1][0, 2] = new_z[1]
            mat[2][0, 0] = new_x[2]
            mat[2][0, 1] = new_y[2]
            mat[2][0, 2] = new_z[2]

        return mat


class Frame:
    def __init__(self, t_sec, t_nsec, nb_markers, marker_list, object_list):
        self.sec = t_sec
        self.nsec = t_nsec
        self.count = nb_markers
        self.marker_list = marker_list
        self.object_list = object_list

    # Can not have any duplicate ids
    def order_markers(self):
        self.marker_list.sort(key=lambda m: m.id, reverse=False)

    def get_marker_by_id(self, id):
        for marker in self.marker_list:
            if marker.id == id:
                return marker

        print "Couldn't find marker with id : ", id
        return None

    def reorder_objects(self):
        new_list = []
        for i in range(NB_HUMAN):
            new_list.append( self.get_object_by_id("Pelvis" + str(i)) )
            new_list.append( self.get_object_by_id("Head" + str(i)) )

        self.object_list = new_list

    def get_object_by_id(self, id):
        for obj in self.object_list:
            if obj.id == id:
                return obj

    def print_marker_ids(self):
        marker_string = '[ '

        for marker in self.marker_list:
            if marker is None:
                marker_string += 'None' + ' '
            else:
                marker_string += str(marker.id) + ' '
        marker_string += ']'
        print marker_string

    def is_similar_frame(self, other):
        if self.count != other.count or self.count != N_MARKERS:
            return False

        for i in range(self.count):

            if self.marker_list[i] is None:
                return False

            if not self.marker_list[i].is_same_marker(other.marker_list[i], THRESHOLD):
                return False

        return True

    # Returns a list of tuples sorted by lowest value (distance)
    # [ ( key, val), ... etc
    def get_distances(self, other):
        dist = {}
        for marker in self.marker_list:
            # dist[marker.id] = random.random()
            dist[marker.id] = marker.get_dist(other)
        return sorted(dist.items(), key=lambda(k, v): v)

    def get_duplicate_ids(self):
        indices = []

        for marker in self.marker_list:
            if not marker == None:
                indices.append(marker.id)

        return [x for x, y in collections.Counter(indices).items() if y > 1]

    def get_threshold(self, id):
        marker = self.marker_list[id]

        return THRESHOLD + THRESHOLD*marker.times_dropped

    def get_new_config_by_distance(self, prev_frame):
        # print "i : ", prev_index+1
        new_markers = [None]*prev_frame.count
        shortest_found = [float('inf')]*prev_frame.count

        # self.print_marker_ids()

        for i, marker in enumerate(self.marker_list):

            dists = prev_frame.get_distances(marker)

            for closest in dists:
                closest_id = closest[0]
                closest_dist = closest[1]

                if shortest_found[closest_id] > closest_dist and closest_dist < prev_frame.get_threshold(closest_id) and closest_dist != 0:
                    new_markers[closest_id] = Marker(closest_id, marker.x, marker.y, marker.z, prev_frame.marker_list[closest_id].name)
                    new_markers[closest_id].times_dropped = 0
                    shortest_found[closest_id] = closest_dist
                    break

        for i in range(0, prev_frame.count):
            if new_markers[i] is None:  # Marker dropped!
                new_markers[i] = prev_frame.get_marker_by_id(i)
                new_markers[i].times_dropped += 1


        temp = Frame(self.sec, self.nsec, len(new_markers), new_markers, self.object_list)
        return temp

    def set_marker_names_by_id(self):
        for marker in self.marker_list:
            if marker.id == 0:
                marker.name = 'ChestFront'
            if marker.id == 1:
                marker.name = 'ChestBack'
            if marker.id == 2:
                marker.name = 'SternumFront'
            if marker.id == 3:
                marker.name = 'SternumBack'
            if marker.id == 4:
                marker.name = 'rShoulderFront'
            if marker.id == 5:
                marker.name = 'rShoulderBack'
            if marker.id == 6:
                marker.name = 'rElbowOuter'
            if marker.id == 7:
                marker.name = 'rElbowInner'
            if marker.id == 8:
                marker.name = 'rWristOuter'
            if marker.id == 9:
                marker.name = 'rWristInner'
            if marker.id == 10:
                marker.name = 'rPalm'
            if marker.id == 11:
                marker.name = 'lShoulderFront'
            if marker.id == 12:
                marker.name = 'lShoulderBack'
            if marker.id == 13:
                marker.name = 'lElbowOuter'
            if marker.id == 14:
                marker.name = 'lElbowInner'
            if marker.id == 15:
                marker.name = 'lWristOuter'
            if marker.id == 16:
                marker.name = 'lWristInner'
            if marker.id == 17:
                marker.name = 'lPalm'

    def set_marker_names(self):
        true_count = NB_MARKERS*NB_HUMAN
        if (self.count is not true_count):
            print "Something went wrong when setting marker names"
            return

        new_id = 0
        for i in range(0, true_count, NB_MARKERS):
            self.marker_list[i].name = 'ChestFront'
            self.marker_list[i+1].name = 'ChestBack'
            self.marker_list[i+2].name = 'SternumFront'
            self.marker_list[i+3].name = 'SternumBack'
            self.marker_list[i+4].name = 'rShoulderFront'
            self.marker_list[i+5].name = 'rShoulderBack'
            self.marker_list[i+6].name = 'rElbowOuter'
            self.marker_list[i+7].name = 'rElbowInner'
            self.marker_list[i+8].name = 'rWristOuter'
            self.marker_list[i+9].name = 'rWristInner'
            self.marker_list[i+10].name = 'rPalm'
            self.marker_list[i+11].name = 'lShoulderFront'
            self.marker_list[i+12].name = 'lShoulderBack'
            self.marker_list[i+13].name = 'lElbowOuter'
            self.marker_list[i+14].name = 'lElbowInner'
            self.marker_list[i+15].name = 'lWristOuter'
            self.marker_list[i+16].name = 'lWristInner'
            self.marker_list[i+17].name = 'lPalm'

    def get_n_closest_markers(self, pelv_frame, n):
        points = []
        dist = {}

        temp_pelv = np.transpose(pelv_frame[:,3])
        pelv = np.array([temp_pelv[0,0], temp_pelv[0,1], temp_pelv[0,2]])

        for marker in self.marker_list:
            # Only consider distance in x and y
            dist[marker.id] = sqrt( (marker.x-pelv[0])**2 + (marker.y-pelv[1])**2 )

        dist_list = sorted(dist.items(), key=lambda(k, v): v)

        # Return the first n points
        for pair in dist_list[:n]:
            # self.print_marker_ids()
            points.append( ( pair[0] ,self.marker_list[pair[0]].array) )

        return points


    def reorder_ids(self):
        for i, marker in enumerate(self.marker_list):
            marker.id = i


class Tracker:

    def __init__(self, marker_topic, object_topic):
        self.bag = subprocess.Popen('rosbag play /home/rafi/logging_two/second/2014-07-24-17-11-06.bag', stdin=subprocess.PIPE, stdout=open(os.devnull, 'w'), shell=True, cwd='./') 
        # self.bag = subprocess.Popen('rosbag play /home/rafi/logging_two/fifth/2014-07-24-17-17-06.bag', stdin=subprocess.PIPE, stdout=open(os.devnull, 'w'), shell=True, cwd='./') 

        self.frames = []
        self.last_frame = None
        self.lock = threading.Lock()

        self.object_q = Queue.Queue()
        self.marker_q = Queue.Queue()

        self.viewer = Drawer()

        if not (marker_topic and object_topic):
            print "At least one topic needed to subscribe to"
            exit(0)
        if marker_topic:
            self.marker_sub = rospy.Subscriber(marker_topic, MocapMarkerArray, self.marker_cb)
        if object_topic:
            self.object_sub = rospy.Subscriber(object_topic, MocapState, self.object_cb)


        # t = threading.Thread(target = rospy.spin)
        # t.start()

        rospy.spin()

        # sleep_rate = rospy.Rate(100.0)
        # while not rospy.is_shutdown():
        #     sleep_rate.sleep()

    def marker_cb(self, msg):
        self.lock.acquire()
        self.marker_q.put(msg)
        # print "Len m_q : ", self.marker_q.qsize()

        if not self.object_q.empty():
            self.make_frame()

        self.lock.release()

        # if self.markers_updated and self.objects_updated:
        #     self.make_frame()

    def object_cb(self, msg):
        
        self.lock.acquire()

        self.object_q.put(msg)
        # print "Len o_q : ", self.object_q.qsize()

        # self.lock.acquire()
        if not self.marker_q.empty():
            self.make_frame()

        self.lock.release()
        # if self.markers_updated and self.objects_updated:
        #     self.make_frame()

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
        frame.reorder_objects()

        if self.last_frame is None:
            self.try_init(frame)
            self.viewer.draw_frame_raw(frame)
            # Draw frame raw
        else:
            frame = frame.get_new_config_by_distance(self.last_frame)
            self.last_frame = frame
            self.viewer.draw_frame_skeleton(frame)

        self.frames.append(frame)

        # print "Num frames : ", len(self.frames), " num obj : ", self.object_q.qsize(), " num mark : ", self.marker_q.qsize()

        # if len(self.frames) == 7000:
        #     self.save_file()


    def save_file(self):
        print "Trying to output new file"

        #  Get the out filename

        m_outpath = 'markers_test.csv'

        o_outpath = 'objects_test.csv'

        print "Trying to normalize ids"
        self.normalize_ids()

        with open(m_outpath, 'w') as m_file:
            with open(o_outpath, 'w') as o_file:
                for frame in self.frames[:]:
                    line_str = ""
                    line_str += str(frame.sec) + ','
                    line_str += str(frame.nsec) + ','
                    line_str += str(frame.count) + ','

                    for marker in frame.marker_list:
                        line_str += str(marker.name) + ','
                        line_str += str(marker.x) + ','
                        line_str += str(marker.y) + ','
                        line_str += str(marker.z) + ','

                    line_str = line_str.rstrip(',')
                    line_str += '\n'
                    m_file.write(line_str)

                    line_str = ""
                    line_str += str(frame.sec) + ','      #TODO should probalby use a different time stamp.  at least align frames by timestamp
                    line_str += str(frame.nsec) + ','
                    line_str += str(len(frame.object_list)) + ','

                    for obj in frame.object_list:
                        line_str += str(obj.id) + ','
                        line_str += str(obj.occluded) + ','
                        line_str += str(obj.x) + ','
                        line_str += str(obj.y) + ','
                        line_str += str(obj.z) + ','
                        line_str += str(obj.r_x) + ','
                        line_str += str(obj.r_y) + ','
                        line_str += str(obj.r_z) + ','
                        line_str += str(obj.r_w) + ','

                    line_str = line_str.rstrip(',')
                    line_str += '\n'
                    o_file.write(line_str)


    def try_init(self, frame):
        if frame.count == ( NB_HUMAN * NB_MARKERS):
            pelv_frames = []
            for object in frame.object_list:
                if 'Pelvis' in object.id and not object.is_occluded():
                    pelv_frames.append(object.get_rot_matrix())

            if len(pelv_frames) is not NB_HUMAN:
                return

            # Get the marker map for each human 
            print "Getting marker name map"
            maps = []

            for pelvis in pelv_frames:
                points = frame.get_n_closest_markers(pelvis, NB_MARKERS)
                maps.append(AssignNames(points, pelvis).assign_marker_names())

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
                frame.set_marker_names()
                frame.reorder_ids()
                self.last_frame = frame

    def reorder_ids(self):
        for frame in self.frames:
            frame.reorder_ids()
            frame.reorder_objects()


    def normalize_ids(self):
        for frame in self.frames:
            for i, marker in enumerate(frame.marker_list):
                marker.id = marker.id%18

    def get_runtime(self):
        start = self.frames[self.first_frame].sec
        end = self.frames[self.last_frame-1].sec

        return end-start

    def on_shutdown(self):
        print "killing bag process"
        subprocess.Popen('kill ' + str(self.bag.pid), shell=True)
        self.save_file()
        print "done"



if __name__ == '__main__':


    rospy.init_node('vicon_tracker')
    marker_topic = rospy.get_param("~markers_topic", "mocap_markers")
    object_topic = rospy.get_param("~objects_topic", "mocap_tracking" )

    t = Tracker(marker_topic, object_topic)

