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
from itertools import permutations

mapping = [-1, 8, 6, 7, 17, 16, 15, 19, 21, 22]

# TorsoX        6           6         PelvisBody        TorsoDummyX
# TorsoY        7           7         TorsoDummyX       TorsoDummyY
# TorsoZ        8           8         TorsoDummyY       TorsoDummyTransX
# rShoulderX    15          15        TorsoDummyTransZ  rShoulderDummyX
# rShoulderZ    16          16        rShoulderDummyX   rShoulderDummyZ
# rShoulderY    17          17        rShoulderDummyZ   rHumerus
# rArmTrans     18          18        rHumerus          rElbowDummy1
# rElbowZ       19          19        rElbowDummy1      rRadius
# rForearmTrans 20          20        rRadius           rWristDummy
# rWristX       21          21        rWristDummy       rWristDummyX
# rWristY       22          22        rWristDummyX      rWristDummyY
# rWristZ       23          23        rWristDummyY      rHand

class Human():

    def __init__(self):

        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.Reset()
        self.env.Load("../ormodels/human_wpi_bio.xml")
        # self.env.Load("../ormodels/human_wpi_new.xml")
        # self.orEnv.Load("robots/pr2-beta-static.zae")

        self.markers = genfromtxt('points.csv', delimiter=',')

        self.motion = genfromtxt('output.ik.csv', delimiter=',')
        self.motion = delete(self.motion, 0, axis=0)

        # Print the array
        # print motion.shape
        # for row in self.motion:
        #     print ' '.join(map(str, row))

        self.human = self.env.GetRobots()[0]
        self.handles = []

        self.offset_torso_shoulder = None
        self.offset_shoulder_elbow = None
        self.offset_elbow_wrist = None
        self.torso_origin = None

        # Set model size from file
        self.set_model_size()
        self.traj = None

        # Torso frame
        # self.t_torso = self.human.GetJoint("TorsoX").GetHierarchyChildLink().GetTransform()
        # print self.t_torso

        self.t_torso = MakeTransform(rodrigues([0, 0, pi]), matrix(self.torso_origin))
        # self.t_torso[0:3, 3] = self.torso_origin

        for j in self.human.GetJoints():
            if j.GetName() == "TorsoX":  # j.GetName() == "rShoulderX" or
                t_link = j.GetHierarchyChildLink().GetTransform()
                self.handles.append(misc.DrawAxes(self.env, t_link, 0.3))

    def play_trajectory(self):

        traj = self.get_trajectory(self.motion)

        # data = traj.GetWaypoint(0)
        # q = traj.GetConfigurationSpecification().ExtractJointValues(data,
        #                                                             self.human,
        #                                                             self.human.GetActiveDOFIndices())
        # self.human.SetDOFValues(q)
        # return

        self.human.GetController().SetPath(traj)
        self.human.WaitForController(0)

    def set_model_size(self):

        # Remove two first columns
        markers = deepcopy(self.markers)
        markers = numpy.delete(markers, s_[0:2], 1)
        markers /= 1000

        (m,) = markers[0].shape
        p = [markers[0][n:n+3] for n in range(0, m, 3)]

        self.torso_origin = p[1]

        p_shoulder_center = array([p[4][0], p[4][1], p[5][2]])
        p_elbow_center = (p[6] + p[7])/2
        p_wrist_center = (p[9] - p[8])/2 + p[8]

        self.offset_torso_shoulder = - self.torso_origin + p_shoulder_center
        self.offset_shoulder_elbow = la.norm(p_shoulder_center - p_elbow_center)
        self.offset_elbow_wrist = la.norm(p_wrist_center - p_elbow_center)

        print self.offset_torso_shoulder
        print self.offset_shoulder_elbow
        print self.offset_elbow_wrist

        self.human.SetDOFValues(self.offset_torso_shoulder.tolist(), [9, 10, 11])
        self.human.SetDOFValues([self.offset_shoulder_elbow], [18])
        self.human.SetDOFValues([self.offset_elbow_wrist], [20])

    def play_markers(self):

        # Remove two first columns
        markers = deepcopy(self.markers)
        markers = numpy.delete(markers, s_[0:2], 1)
        markers /= 1000

        # time for playing
        t = 0.0
        alpha = 4  # Time scaling

        for i, points in enumerate(markers):

            del self.handles[:]

            (m,) = points.shape  # number of values in the marker set

            colors = []
            nb_points = len(range(0, m, 3))
            for n in linspace(0.0, 1.0, num=nb_points):
                colors.append((float(n)*1, (1-float(n))*1, 0))

            inv_torso = la.inv(self.t_torso)
            points_3d = [points[n:n+3] for n in range(0, m, 3)]

            # print inv_torso

            points_draw = []

            for j, p in enumerate(points_3d):
                points_3d[j] = array(array(inv_torso).dot(array(append(p, 1.0))))[0:3]

            points_3d = array(points_3d)
            points_3d = squeeze(points_3d)

            # print points_3d
            # print points.shape
            # print points_3d.shape

            self.handles.append(self.env.plot3(points=points_3d, pointsize=0.02, colors=array(colors), drawstyle=1))
            # , color=array((1, 0, 0)), drawstyle=1))
            dt = self.markers.item((i, 1)) - t  # self.markers(1, i) is time
            t = self.markers.item((i, 1))

            if self.traj is not None:
                q = self.traj.Sample(t)  # get configuration
                self.human.SetDOFValues(q[0:self.human.GetDOF()])
            # print t
            time.sleep(alpha*dt)

    def compute_dist_to_points(self):

        markers = deepcopy(self.markers)
        markers = numpy.delete(markers, s_[0:2], 1)
        markers /= 1000
        (m,) = markers[0].shape
        p = [markers[0][n:n+3] for n in range(0, m, 3)]

        dist = 0.0

        p_shoulder_center = array([p[4][0], p[4][1], p[5][2]])
        p_elbow_center = (p[6] + p[7])/2
        p_wrist_center = (p[9] - p[8])/2 + p[8]

        inv_torso = la.inv(self.t_torso)

        for j in self.human.GetJoints():
            p_link = j.GetHierarchyChildLink().GetTransform()[0:3, 3]
            if j.GetName() == "rShoulderX":
                dist += la.norm(p_link - array(inv_torso).dot(append(p_shoulder_center, 1))[0:3])
            if j.GetName() == "rElbowZ":
                dist += la.norm(p_link - array(inv_torso).dot(append(p_elbow_center, 1))[0:3])
            if j.GetName() == "rWristX":
                dist += la.norm(p_link - array(inv_torso).dot(append(p_wrist_center, 1))[0:3])
        return dist

    def get_configuration(self, q):
        wp = self.human.GetDOFValues()
        for i, dof in enumerate(q):
            if mapping[i] >= 0:
                wp[mapping[i]] = dof * pi / 180
        return wp

    def get_trajectory(self, motion):

        config_spec = self.human.GetActiveConfigurationSpecification()
        g = config_spec.GetGroupFromName('joint_values')
        g.interpolation = 'linear'
        config_spec = ConfigurationSpecification()
        config_spec.AddGroup(g)
        config_spec.AddDeltaTimeGroup()

        self.traj = RaveCreateTrajectory(self.human.GetEnv(), '')
        self.traj.Init(config_spec)
        t = 0.0
        alpha = 4  # Time scaling

        for q in motion:
            dt = q[0] - t  # q[0] is time
            t = q[0]
            wp = append(self.get_configuration(q), alpha*dt)
            self.traj.Insert(self.traj.GetNumWaypoints(), wp)

        return self.traj


if __name__ == "__main__":

    h = Human()
    # print "Press return to play trajectory."
    # sys.stdin.readline()

    h.get_trajectory(h.motion)

    while True:
        h.play_markers()
        print "Press return to exit."
        sys.stdin.readline()

    # h.human.SetDOFValues([90 * pi / 180], [16])  # Elevation
    # print "Press return to play trajectory."
    # sys.stdin.readline()
    #
    # h.human.SetDOFValues([90 * pi / 180], [15])  # Plane of Elevation
    # print "Press return to play trajectory."
    # sys.stdin.readline()

    # h.human.SetDOFValues([-30 * pi / 180], [17]) # In rotation
    # print "Press return to play trajectory."
    # sys.stdin.readline()
    #
    # h.human.SetDOFValues([30 * pi / 180], [17]) # In rotation
    # print "Press return to play trajectory."
    # sys.stdin.readline()

    # h.human.SetDOFValues([-30 * pi / 180], [17]) # In rotation
    # print "Press return to play trajectory."
    # sys.stdin.readline()
    #
    # h.human.SetDOFValues([30 * pi / 180], [17]) # In rotation
    # print "Press return to play trajectory."
    # sys.stdin.readline()

    # while True:
    #     h.play_trajectory()
    #     print "Press return to exit."
    #     sys.stdin.readline()

    for idx in list(permutations([15, 16, 17])):
        mapping = [-1, 8, 6, 7, idx[0], idx[1], idx[2], 19, 21, 22]
        # h.play_trajectory()
        q = h.get_configuration(h.motion[0])
        h.human.SetDOFValues(q)
        print mapping
        print h.compute_dist_to_points()