#!/usr/bin/python

# Rafi Hayne

from TransformMatrix import *
import csv
import numpy as np
import os
import time
import math
import sys
import copy
from MocapCommon import *
from MarkerMapper import *

class MarkerFixer:

    def __init__(self, m_filepath, o_filepath):
        self.m_filepath = m_filepath
        self.o_filepath = o_filepath
        self.frames = []
        self.max_markers = None
        self.first_frame = None
        self.last_frame = None

    def load_file(self):
        print "Trying to open file"
        # global NB_HUMAN # TODO fix global to be class member

        marker_file = []
        object_file = []

        with open(self.m_filepath, 'r') as m_file:
            with open(self.o_filepath, 'r') as o_file:

                marker_file = [row for row in csv.reader(m_file, delimiter=',')]
                object_file = [row for row in csv.reader(o_file, delimiter=',')]

        nb_lines = min(len(marker_file), len(object_file))
        self.last_frame = nb_lines

        for row in range(nb_lines):

            markers = []
            objects = []

            m_cells = marker_file[row]
            o_cells = object_file[row]

            # Load Objects
            count = int(o_cells[2])

            # Assuming only using Pelv/Head objects per person and nothing else in the scene
            # NB_HUMAN = count/2

            for i in range(3, count*9, 9):
                name = str(o_cells[i])
                occluded = int(o_cells[i+1])
                x = float(o_cells[i+2])
                y = float(o_cells[i+3])
                z = float(o_cells[i+4])
                r_x = float(o_cells[i+5])
                r_y = float(o_cells[i+6])
                r_z = float(o_cells[i+7])
                r_w = float(o_cells[i+8])


                object = Object( name, occluded, x, y, z, r_x, r_y, r_z, r_w )
                objects.append(object)

            # Load Markers
            sec = float(m_cells[0])
            nsec = float(m_cells[1])
            count = int(m_cells[2])

            for i in range(3, count*4, 4):
                id = int(m_cells[i])
                x = float(m_cells[i+1])
                y = float(m_cells[i+2])
                z = float(m_cells[i+3])

                marker = Marker(id, x, y, z)
                markers.append(marker)


            self.frames.append( Frame(sec, nsec, count, markers, objects) )

        print "# configs loaded : " + str(len(self.frames))

    def save_file(self):
        print "Trying to output new file"

        #  Get the out filename
        dir, path = os.path.split(self.m_filepath)
        name, type = path.rsplit('.', 1)
        m_outpath = dir+'/'+name + '_fixed.'+type
        drop_outpath = dir+'/'+name + '_dropped.'+type

        dir, path = os.path.split(self.o_filepath)
        name, type = path.rsplit('.', 1)
        o_outpath = dir+'/'+name + '_fixed.'+type

        print "saving file : ", m_outpath

        # No need to normalize ids.  we output marker names
        # print "Trying to normalize ids"
        # self.normalize_ids()

        with open(m_outpath, 'w') as m_file:
            with open(o_outpath, 'w') as o_file:
                with open(drop_outpath, 'w') as d_file:
                    for frame in self.frames[self.first_frame:self.last_frame]:
                        drop_str = ''

                        line_str = ""
                        line_str += str(frame.sec) + ','
                        line_str += str(frame.nsec) + ','
                        line_str += str(frame.count) + ','

                        for marker in frame.marker_list:
                            drop_str += str(marker.times_dropped) + ','
                            line_str += str(marker.name) + ','
                            line_str += str(marker.x) + ','
                            line_str += str(marker.y) + ','
                            line_str += str(marker.z) + ','

                        line_str = line_str.rstrip(',')
                        line_str += '\n'
                        m_file.write(line_str)

                        drop_str = drop_str.rstrip(',')
                        drop_str += '\n'
                        d_file.write(drop_str)

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

    def get_average_position(self):
        m_tot = np.array( [0, 0, 0] )
        m_count = 0.0

        for frame in self.frames:
            for marker in frame.marker_list:
                m_tot += marker.numpy()
                m_count += 1

        print "trying to finding avg pos"

        return m_tot/m_count

    def filter_threshold_inside(self, point, threshold):
        nb_removed = 0
        avg = Marker(0, point[0], point[1], point[2])

        print "Trying to filter markers ", threshold, "m within : ", point
        for frame in self.frames:

            remove_list = []

            for marker in frame.marker_list:
                if math.sqrt(avg.get_dist(marker)) < threshold:
                    remove_list.append(marker)
                    frame.count -= 1
                    nb_removed += 1

            for r_marker in remove_list:
                frame.marker_list.remove(r_marker)

        print "Filtered ", nb_removed, ' markers'

    def filter_pillar(self):
        nb_removed = 0
        origin = Marker(0,0,0,0)

        for frame in self.frames:

            remove_list = []

            for marker in frame.marker_list:
                x_dist = (origin.x - marker.x)
                y_dist = (origin.y - marker.y)
                dist = np.array([x_dist,y_dist, marker.z])

                if np.linalg.norm(dist) < 1:
                    # config.marker_list = config.marker_list.remove(marker)
                    remove_list.append(marker)
                    frame.count -= 1
                    nb_removed += 1
            for r_marker in remove_list:
                frame.marker_list.remove(r_marker)

        print "Filtered ", nb_removed, ' markers'

    def filter_threshold_outside(self, point, threshold):
        nb_removed = 0
        avg = Marker(0, point[0], point[1], point[2])

        print "Trying to filter markers ", threshold, "m outside : ", point
        for frame in self.frames:

            remove_list = []

            for marker in frame.marker_list:
                if math.sqrt(avg.get_dist(marker)) > threshold:
                    remove_list.append(marker)
                    frame.count -= 1
                    nb_removed += 1

            for r_marker in remove_list:
                frame.marker_list.remove(r_marker)

        print "Filtered ", nb_removed, ' markers'

    def filter_negative_x(self):
        nb_removed = 0

        for frame in self.frames:
            remove_list = []

            for marker in frame.marker_list:
                if marker.x < 0.6:
                    remove_list.append(marker)
                    frame.count -= 1
                    nb_removed += 1

            for r_marker in remove_list:
                frame.marker_list.remove(r_marker)

        print "Filtered ", nb_removed, ' markers'

    def init_first_frame(self):
        # TODO Get # Human from objects
        # TODO Check if marker and object timestamps for first_frame match up

        # Find first usable config
        for i, frame in enumerate(self.frames):
            if frame.count == ( NB_HUMAN * NB_MARKERS):

                pelv_frames = []
                for object in frame.object_list:
                    if object and 'Pelvis' in object.id and not object.is_occluded():
                        pelv_frames.append(object.get_rot_matrix())

                if len(pelv_frames) is not NB_HUMAN:
                    continue

                # Get the marker map for each human
                print "Getting marker name map"
                maps = []

                for pelvis in pelv_frames:
                    points = frame.get_n_closest_markers(pelvis, NB_MARKERS)
                    maps.append(MarkerMapper(points, pelvis).assign_marker_names(ELBOW_PADS, RARM_ONLY))

                # Reorder markers according to map
                new_marker_list = []
                for map in maps:
                    for id in map:
                        new_marker_list.append(frame.marker_list[id])

                print "Concatenated marker list"
                frame.marker_list = new_marker_list
                frame.print_marker_ids()

                if frame.get_duplicate_ids() or len(frame.marker_list) != frame.count:
                    continue

                else:
                    self.first_frame = i
                    break

        if self.first_frame is not None:
            frame = self.frames[self.first_frame]
            # Now all markers are in the proper order, but names aren't set and ids are out of order
            print "Assign marker names"
            frame.set_marker_names(NB_HUMAN, NB_MARKERS, ELBOW_PADS, RARM_ONLY)

            # Put all ids in ascending order
            frame.reorder_ids()
            print "Successfully found good first frame : ", self.first_frame
            frame.print_marker_ids()

        else:
            print "Couldn't find a usable first frame"


    def track_indices(self):
        if self.first_frame != None:
            prev = self.frames[self.first_frame]
            for i in range(self.first_frame+1, self.last_frame):
                curr = self.frames[i]
                new_frame = curr.get_new_config_by_distance(prev, THRESHOLD)
                self.frames[i] = new_frame
                prev = new_frame

    def reorder_ids(self):
        for frame in self.frames:
            frame.reorder_ids()
            frame.reorder_objects(NB_HUMAN, ELBOW_PADS, RARM_ONLY)

    def smooth_markers(self, size):
        if size%2 == 0:
            print "Window size can't be even"
            return

        new_frames = self.frames[:]

        window = self.frames[self.first_frame:self.first_frame+size]

        for i in range( self.first_frame + int(floor(size/2)), self.last_frame-int(floor(size/2))):
            new_markers = []
            for m, marker in enumerate(self.frames[i].marker_list):
                avg = np.array([0.0,0.0,0.0])

                for a_frame in window:
                    # print "frame : ", i, "m : ", m, "len frame : ", len(a_frame.marker_list)
                    avg += a_frame.marker_list[m].array

                avg = avg/float(size)

                # self.frames[i].marker_list[m].array = avg
                # self.frames[i].marker_list[m].x = avg[0]
                # self.frames[i].marker_list[m].y = avg[1]
                # self.frames[i].marker_list[m].z = avg[2]

                temp_marker = self.frames[i].marker_list[m]
                temp_marker.array = avg
                temp_marker.x = avg[0]
                temp_marker.y = avg[1]
                temp_marker.z = avg[2]

                new_markers.append(temp_marker)

            new_frames[i].marker_list = new_markers

            window = window[1:] + [self.frames[i+1]]

        self.frames = new_frames

    def normalize_ids(self):
        for frame in self.frames:
            for i, marker in enumerate(frame.marker_list):
                marker.id = marker.id%18

    def get_runtime(self):
        start = self.frames[self.first_frame].get_time()
        end = self.frames[self.last_frame-1].get_time()

        return end-start

    def calc_stats(self):
        max_dist = float('-inf')
        two_count = 0
        three_count = 0
        five_count = 0

        for i in range(self.first_frame, self.last_frame-1):
            curr = self.frames[i]
            next = self.frames[i+1]

            dists = curr.get_dist_between_frames(next)
            for dist in dists:
                if dist > max_dist:
                    max_dist = dist

                if dist >= 0.05:
                    five_count += 1
                    continue
                if dist >= 0.03:
                    three_count += 1
                    continue
                if dist >= 0.02:
                    two_count += 1
                    continue

        print "# of deltas > 2cm : ", two_count, " > 3cm : ", three_count, " > 5cm : ", five_count
        print "Max distance : ", max_dist

if __name__ == '__main__':

    THRESHOLD   = 0.0025
    ELBOW_PADS  = True
    RARM_ONLY   = True
    NB_MARKERS  = get_nb_markers(ELBOW_PADS, RARM_ONLY)
    NB_HUMAN    = 2


    # f = MarkerFixer('/home/rafi/logging_six/2/markers.csv', '/home/rafi/logging_six/2/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_five/1/markers.csv', '/home/rafi/logging_five/1/objects.csv')

    # f = MarkerFixer('/home/rafi/logging_three/first/markers.csv', '/home/rafi/logging_three/first/objects.csv')
    # f = MarkerFixer('/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[0000-1700]markers.csv', '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[0000-1700]objects.csv')
    # f = MarkerFixer('/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[1750-3200]markers.csv', '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[1750-3200]objects.csv')

    # f = MarkerFixer('/home/rafi/logging_nine/2/[1000-3900]markers.csv', '/home/rafi/logging_nine/2/[1000-3900]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[5900-9000]markers.csv', '/home/rafi/logging_nine/2/[5900-9000]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[11700-14800]markers.csv', '/home/rafi/logging_nine/2/[11700-14800]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[22400-25300]markers.csv', '/home/rafi/logging_nine/2/[22400-25300]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[28300-30800]markers.csv', '/home/rafi/logging_nine/2/[28300-30800]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[33000-35700]markers.csv', '/home/rafi/logging_nine/2/[33000-35700]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[37900-40400]markers.csv', '/home/rafi/logging_nine/2/[37900-40400]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[42600-44700]markers.csv', '/home/rafi/logging_nine/2/[42600-44700]objects.csv')

    # Trials
    # f = MarkerFixer('/home/rafi/logging_five/1/markers.csv', '/home/rafi/logging_five/1/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_five/2/markers.csv', '/home/rafi/logging_five/2/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_five/3/markers.csv', '/home/rafi/logging_five/3/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_five/4/markers.csv', '/home/rafi/logging_five/4/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_six/1/markers.csv', '/home/rafi/logging_six/1/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_six/2/markers.csv', '/home/rafi/logging_six/2/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_six/3/markers.csv', '/home/rafi/logging_six/3/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_six/4/markers.csv', '/home/rafi/logging_six/4/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_six/5/markers.csv', '/home/rafi/logging_six/5/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_six/6/markers.csv', '/home/rafi/logging_six/6/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_seven/1/markers.csv', '/home/rafi/logging_seven/1/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_seven/2/markers.csv', '/home/rafi/logging_seven/2/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_seven/3/markers.csv', '/home/rafi/logging_seven/3/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_seven/4/markers.csv', '/home/rafi/logging_seven/4/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_seven/5/markers.csv', '/home/rafi/logging_seven/5/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_seven/6/markers.csv', '/home/rafi/logging_seven/6/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_seven/7/markers.csv', '/home/rafi/logging_seven/7/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_seven/8/markers.csv', '/home/rafi/logging_seven/8/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_seven/9/markers.csv', '/home/rafi/logging_seven/9/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_seven/10/markers.csv', '/home/rafi/logging_seven/10/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_seven/11/markers.csv', '/home/rafi/logging_seven/11/objects.csv')


    # f = MarkerFixer('/home/rafi/logging_eight/1/markers.csv', '/home/rafi/logging_eight/1/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_eight/2/markers.csv', '/home/rafi/logging_eight/2/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_eight/3/markers.csv', '/home/rafi/logging_eight/3/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_eight/4/markers.csv', '/home/rafi/logging_eight/4/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_eight/5/markers.csv', '/home/rafi/logging_eight/5/objects.csv')
    # f = MarkerFixer('/home/rafi/logging_eight/6/markers.csv', '/home/rafi/logging_eight/6/objects.csv')

    # f = MarkerFixer('/home/rafi/logging_nine/1/markers.csv', '/home/rafi/logging_nine/1/objects.csv')

    # f = MarkerFixer('/home/rafi/logging_nine/2/[1000-3900]markers.csv', '/home/rafi/logging_nine/2/[1000-3900]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[5900-9000]markers.csv', '/home/rafi/logging_nine/2/[5900-9000]objects.csv')
    f = MarkerFixer('/home/rafi/logging_nine/2/[11700-14800]markers.csv', '/home/rafi/logging_nine/2/[11700-14800]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[22400-25300]markers.csv', '/home/rafi/logging_nine/2/[22400-25300]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[28300-30800]markers.csv', '/home/rafi/logging_nine/2/[28300-30800]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[33000-35700]markers.csv', '/home/rafi/logging_nine/2/[33000-35700]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[37900-40400]markers.csv', '/home/rafi/logging_nine/2/[37900-40400]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[42600-44700]markers.csv', '/home/rafi/logging_nine/2/[42600-44700]objects.csv')

    try:
        with Timer() as t:
            f.load_file()

            # Filter bad markers

            # avg = f.get_average_position()
            # f.filter_threshold_outside(avg, 1.8)
            # f.filter_negative_x()
            # f.filter_pillar()

            # Reorder marker ids to fill gaps
            f.reorder_ids()

            print "Try to find start frame"
            f.init_first_frame()

            print "starting to track ids"
            f.track_indices()

            print "Calculating statistics : "
            f.calc_stats()

            # print "Trying to smooth markers"
            # f.smooth_markers(5)

            # print "Calculating statistics after smoothing : "
            # f.calc_stats()

            f.save_file()

            t.file_runtime = f.get_runtime()
    finally:
        if t.file_runtime:
            print 'Marker matching took,', t.interval, ' sec, ', (t.interval/t.file_runtime)*100, '% of total runtime'
        else:
            print 'Marker matching took %.03f sec.' % t.interval