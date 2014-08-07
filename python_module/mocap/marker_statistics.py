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

THRESHOLD   = 0.0025
NB_MARKERS  = 18
NB_HUMAN    = 2

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
        return np.linalg.norm(self.array-other.array)


class Frame:
    def __init__(self, t_sec, t_nsec, nb_markers, marker_list):
        self.sec = t_sec
        self.nsec = t_nsec
        self.count = nb_markers
        self.marker_list = marker_list

    def get_marker_by_id(self, id):
        for marker in self.marker_list:
            if marker.id == id:
                return marker

        print "Couldn't find marker with id : ", id
        return None

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

        print dist
        # return sorted(dist.items(), key=lambda(k, v): v)

    def get_index_list_by_id(self, id):
        index_list = []

        for i,marker in enumerate(self.marker_list):
            if marker.id == id:
                index_list.append(i)

        return index_list


    def get_dist_between_frames(self, next):
        dist_list = []

        for i, marker in enumerate(self.marker_list):
            dist_list.append(next.marker_list[i].get_dist(marker))

        return dist_list



class Statistics:

    def __init__(self, m_filepath):
        self.m_filepath     = m_filepath
        self.frames         = []
        self.max_markers    = None
        self.first_frame    = None
        self.last_frame     = None

    def load_file(self):
        print "Trying to open file"

        marker_file = []

        with open(self.m_filepath, 'r') as m_file:
            marker_file = [row for row in csv.reader(m_file, delimiter=',')]

        nb_lines            = len(marker_file)
        self.last_frame     = nb_lines

        for row in range(nb_lines):
            markers = []
            m_cells = marker_file[row]

            # Load Markers
            sec     = float(m_cells[0])
            nsec    = float(m_cells[1])
            count   = int(m_cells[2])

            nb_seen = 0
            for i in range(3, count*4, 4):
                id          = nb_seen
                name        = str(m_cells[i])
                x           = float(m_cells[i+1])
                y           = float(m_cells[i+2])
                z           = float(m_cells[i+3])
                nb_seen     += 1

                marker = Marker(id, x, y, z)
                markers.append(marker)


            self.frames.append( Frame(sec, nsec, count, markers) )

        print "# configs loaded : " + str(len(self.frames))

    def get_average_position(self):
        m_tot = np.array( [0, 0, 0] )
        m_count = 0.0

        for frame in self.frames:
            for marker in frame.marker_list:
                m_tot += marker.numpy()
                m_count += 1

        print "trying to finding avg pos"

        return m_tot/m_count



if __name__ == '__main__':
    s = Statistics('/home/rafi/workspace/hrics-or-plugins/examples/markers_fixed.csv')

    max_dist = float('-inf')
    two_count = 0
    three_count = 0
    five_count = 0


    s.load_file()
    for i in range(0, s.last_frame-1):
        curr = s.frames[i]
        next = s.frames[i+1]

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

    print "Raw fixed"
    print "# of deltas > 2cm : ", two_count, " > 3cm : ", three_count, " > 5cm : ", five_count
    print "Max distance : ", max_dist

    s = Statistics('/home/rafi/workspace/hrics-or-plugins/examples/markers_smoothed.csv')

    max_dist = float('-inf')
    two_count = 0
    three_count = 0
    five_count = 0


    s.load_file()
    for i in range(0, s.last_frame-1):
        curr = s.frames[i]
        next = s.frames[i+1]

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

    print "smoothed"
    print "# of deltas > 2cm : ", two_count, " > 3cm : ", three_count, " > 5cm : ", five_count
    print "Max distance : ", max_dist

