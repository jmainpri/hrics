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
from copy import deepcopy
from numpy import *
from numpy import linalg as la
from TransformMatrix import *
from rodrigues import *
from subprocess import call

mapping = [-1, 8, 7, 6, 18, 17, 16, 20, 24, 23, 22]

# Joint and link names
# -------------------------------------------------------------------------
# TorsoX        6           6         PelvisBody        TorsoDummyX
# TorsoY        7           7         TorsoDummyX       TorsoDummyY
# TorsoZ        8           8         TorsoDummyY       TorsoDummyTransX
# xTorsoTrans   9           9         TorsoDummyTransX  TorsoDummyZ
# yTorsoTrans   10          10        TorsoDummyTransX  TorsoDummyTransY
# zTorsoTrans   11          11        TorsoDummyTransY  TorsoDummyTransZ
# rShoulderX    16          16        TorsoDummyTransZ  rShoulderDummyX
# rShoulderZ    17          17        rShoulderDummyX   rShoulderDummyZ
# rShoulderY    18          18        rShoulderDummyZ   rHumerus
# rArmTrans     19          19        rHumerus          rElbowDummy1
# rElbowZ       20          20        rElbowDummy1      rRadius
# rForearmTrans 21          21        rRadius           rWristDummy
# rWristX       22          22        rWristDummy       rWristDummyX
# rWristY       23          23        rWristDummyX      rWristDummyY
# rWristZ       24          24        rWristDummyY      rHand

# Reminder of marker order
# -------------------------------------------------------------------------
# 0 xyphoid process
# 1 T8
# 2 sternal notch
# 3 C7
# 4 Acromion process
# 5 Glenohumeral cntr of rot. (post)
# 6 Medial epicondyle
# 7 lateral epicondyle
# 8 ulnar styloid
# 9 radial styloid
# 10 2nd metacarpal head


class BioHumanIk():

    def __init__(self):

        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.Reset()
        self.env.Load("../ormodels/human_wpi_bio.xml")

        self.human = self.env.GetRobots()[0]
        self.handles = []

        # Should store the marker set
        self.markers = None
        self.q = None

        # Set torso at origin of the scene
        self.offset_pelvis_torso = self.human.GetJoint("TorsoX").GetHierarchyChildLink().GetTransform()[0:3, 3]
        # print self.offset_pelvis_torso

        self.offset_torso_shoulder = None
        self.offset_shoulder_elbow = None
        self.offset_elbow_wrist = None
        self.torso_origin = None
        self.t_torso = eye(4)

        self.human.SetTransform(array(MakeTransform(eye(3), matrix(-self.offset_pelvis_torso))))

    # Marker array should be the size of the number of markers
    # the markers should be ordered as described in the reminder
    def set_markers(self, markers):

        self.markers = markers

    def get_markers_in_torso_frame(self):

        inv_torso = la.inv(self.t_torso)
        points_3d = array([0, 0, 0])

        for i, p in enumerate(self.markers):
            points_3d[i] = array(array(inv_torso).dot(array(append(p, 1.0))))[0:3]

        return points_3d

    def set_model_size(self):

        self.torso_origin = (self.markers[0] + self.markers[1])/2
        self.t_torso = MakeTransform(rodrigues([0, 0, pi]), matrix(self.torso_origin))

        self.torso_origin = (self.markers[0] + (self.markers[1]))/2
        p_shoulder_center = array([self.markers[4][0], self.markers[4][1], self.markers[5][2]])
        p_elbow_center = (self.markers[6] + self.markers[7])/2
        p_wrist_center = (self.markers[9] - self.markers[8])/2 + self.markers[8]

        self.offset_torso_shoulder = p_shoulder_center - self.torso_origin
        self.offset_shoulder_elbow = la.norm(p_shoulder_center - p_elbow_center)
        self.offset_elbow_wrist = la.norm(p_wrist_center - p_elbow_center)

        # Get shoulder center in the global frame
        t_torso = self.t_torso
        inv_torso = la.inv(t_torso)
        offset_torso = array(array(inv_torso).dot(append(p_shoulder_center, 1)))[0:3]

        # Get shoulder center in the torso frame after rotation
        t_torso = self.human.GetJoint("TorsoZ").GetHierarchyChildLink().GetTransform()
        inv_torso = la.inv(t_torso)
        offset_torso = array(array(inv_torso).dot(append(offset_torso, 1)))[0:3]

        self.human.SetDOFValues([offset_torso[0]], [9])
        self.human.SetDOFValues([offset_torso[1]], [10])
        self.human.SetDOFValues([offset_torso[2]], [11])
        self.human.SetDOFValues([self.offset_shoulder_elbow], [19])
        self.human.SetDOFValues([self.offset_elbow_wrist], [21])

    def draw_markers(self):

        points = self.get_markers_in_torso_frame()

        colors = []
        nb_points = len(points)
        for n in linspace(0.0, 1.0, num=nb_points):
            colors.append((float(n)*1, (1-float(n))*1, 0))

        points_3d = squeeze(points)

        self.handles.append(self.env.plot3(points=points_3d, pointsize=0.02, colors=array(colors), drawstyle=1))

        if self.traj is not None:  # Set robot to trajectory configuration if it exists
            q_cur = self.get_human_configuration()
            self.human.SetDOFValues(q_cur[0:self.human.GetDOF()])
            # print "(plane of elevation, ", q_cur[16]*180/pi, " , elevation, ", q_cur[17]*180/pi, \
            #     " , axial rotation, ", q_cur[18]*180/pi, ")"

        # draws center of joints points
        self.set_model_size()
        # sys.stdin.readline()

        del self.handles[:]

    # Map the joint angles and set to radians
    def get_human_configuration(self):

        # Save markers to file

        # Call to ik
        call(["octave", "matlab/run_one_ik.m"])

        motion = genfromtxt('./matlab/outputik.csv', delimiter=',')
        motion = delete(motion, 0, axis=0)  # Remove first row...

        for configuration in motion:  # motion should be one row. otherwise take the last element
            self.q = self.human.GetDOFValues()
            for i, dof in enumerate(configuration):
                if mapping[i] >= 0:
                    self.q[mapping[i]] = dof * pi / 180
                if mapping[i] == 16:
                    self.q[mapping[i]] = -self.q[mapping[i]]
                if mapping[i] == 17:
                    self.q[mapping[i]] = -self.q[mapping[i]]
                if mapping[i] == 18:
                    self.q[mapping[i]] = -self.q[mapping[i]]
        return self.q

if __name__ == "__main__":

    # Load markers from file
    raw_markers = genfromtxt('points.csv', delimiter=',')
    raw_markers = delete(raw_markers, 0, axis=0)  # Remove first row
    raw_markers = delete(raw_markers, s_[0:2], 1)  # Remove two columns
    raw_markers[0] /= 1000  # Set markers in meters
    (m,) = raw_markers[0].shape  # number of values in the marker set
    markers = [raw_markers[0][n:n+3] for n in range(0, m, 3)]  # separate in 3d arrays
    print markers

    h = BioHumanIk()
    h.set_markers(markers)