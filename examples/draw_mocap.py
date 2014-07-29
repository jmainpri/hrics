#!/usr/bin/env python
# Copyright (c) 2013 Worcester Polytechnic Institute
#   Author: Jim Mainrpice <jmainprice@wpi.edu>
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor Carnegie Mellon University,
#       nor the names of their contributors, may be used to endorse or
#       promote products derived from this software without specific prior
#       written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
#   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# -*- coding: utf-8 -*-

from openravepy import *
import os
import sys
import time
import csv
from copy import deepcopy
import numpy as np
from numpy import linalg as la
from TransformMatrix import *
from rodrigues import *
from subprocess import call

NB_MARKERS = 18
NB_HUMAN = 1 #  TODO Read in from file like in fix_marker_id

class DrawMarkers():

    def __init__(self):

        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.Reset()
        self.env.Load("../ormodels/human_wpi_bio.xml")

        self.human = self.env.GetRobots()[0]
        self.handles = []

        # Should store the marker set
        self.frames = []

    def load_file(self, m_filepath, o_filepath):
        print "Trying to open file"

        marker_file = []
        object_file = []

        with open(m_filepath, 'r') as m_file:
            with open(o_filepath, 'r') as o_file:

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

            for i in range(3, count*9, 9):
                name = str(o_cells[i])
                occluded = int(o_cells[i+1])
                x = float(o_cells[i+2])
                y = float(o_cells[i+3])
                z = float(o_cells[i+4])


                object = Object( name, occluded, x, y, z)
                objects.append(object)

            # Load Markers
            sec = float(m_cells[0])
            nsec = float(m_cells[1])
            count = int(m_cells[2])


            nb_seen = 0
            for i in range(3, count*4, 4):
                id = nb_seen
                name = str(m_cells[i])
                x = float(m_cells[i+1])
                y = float(m_cells[i+2])
                z = float(m_cells[i+3])
                nb_seen += 1

                marker = Marker(id, x, y, z, name)
                markers.append(marker)


            self.frames.append( Frame(sec, nsec, count, markers, objects) )

        print "# configs loaded : " + str(len(self.frames))

    def draw_center_points(self):
        # for frame in self.frames:
        prev_time = self.frames[0].get_time()

        for frame in self.frames:
            curr_time = frame.get_time()
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


            dt = curr_time - prev_time
            prev_time = curr_time
            time.sleep(dt)

            del self.handles[:]


    def draw_frames_raw(self):
        # for frame in self.frames:
        prev_time = self.frames[0].get_time()

        for frame in self.frames:
            curr_time = frame.get_time()

            point_list = []

            for m in frame.marker_list:
                point_list.append(m.array)
            for o in frame.object_list:
                point_list.append(o.array)

            self.draw_points(np.array(point_list))


            dt = curr_time - prev_time
            prev_time = curr_time
            time.sleep(dt)

            del self.handles[:]


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

    def numpy(self):
        return self.array

class Object:
    def __init__(self, id, occluded, x, y, z):
        self.id = id
        self.occluded = occluded
        self.x = x
        self.y = y
        self.z = z
        self.array = np.array([self.x, self.y, self.z])

    def is_occluded(self):
        return self.occluded


class Frame:
    def __init__(self, t_sec, t_nsec, nb_markers, marker_list, object_list):
        self.sec = t_sec
        self.nsec = t_nsec
        self.count = nb_markers
        self.marker_list = marker_list
        self.object_list = object_list

    def get_marker_by_id(self, id):
        for marker in self.marker_list:
            if marker.id == id:
                return marker

        print "Couldn't find marker with id : ", id
        return None

    def get_time(self):
        return self.sec + (self.nsec/1000000000)

if __name__ == "__main__":

    d = DrawMarkers()
    # d.load_file('/home/rafi/workspace/hrics-or-plugins/examples/markers_smoothed.csv', '/home/rafi/logging_data/third/objects.csv')
    # d.load_file('/home/rafi/workspace/hrics-or-plugins/examples/markers_fixed.csv', '/home/rafi/logging_data/fourth/objects_fixed.csv')
    d.load_file('/home/rafi/workspace/hrics-or-plugins/examples/markers_test.csv', '/home/rafi/workspace/hrics-or-plugins/examples/objects_test.csv')
    sys.stdin.readline()
    # d.draw_center_points()
    d.draw_frames_raw()