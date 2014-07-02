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
from numpy import *
from TransformMatrix import *
from rodrigues import *

mapping = [-1, 6, 7, 8, 12, 13, 14, 16, 17, 18]


class human():

    def __init__(self):
        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.Reset()
        self.env.Load("../ormodels/human_wpi_bio.xml")
        # self.env.Load("../ormodels/human_wpi_new.xml")
        # self.orEnv.Load("robots/pr2-beta-static.zae")

        self.markers = numpy.genfromtxt('points.csv', delimiter=',')

        self.motion = numpy.genfromtxt('output.ik.csv', delimiter=',')
        self.motion = numpy.delete(self.motion, 0, axis=0)

        # Print the array
        # print motion.shape
        # for row in self.motion:
        #     print ' '.join(map(str, row))

        self.human = self.env.GetRobots()[0]
        self.handles = []

        for j in self.human.GetJoints():
            t_link = j.GetHierarchyChildLink().GetTransform()
            self.handles.append(misc.DrawAxes(self.env, t_link, 0.3))

    # def SetConfiguration(row):

    def PlayTrajectory(self):

        traj = self.GetTrajectory(self.motion)
        self.human.GetController().SetPath(traj)
        self.human.WaitForController(0)

    def PlayMarkers(self):

        # Remove two first columns
        markers = numpy.delete(self.markers, s_[0:2], 1)
        markers /= 1000

        for points in markers:
            # (m,) = points.shape
            # p = [points[n:n+3] for n in range(0, m, 3)]
            colors = points
            self.handles.append(self.env.plot3(points=points, pointsize=5.0))
            # , color=array((1, 0, 0)), drawstyle=1))
            time.sleep(0.02)
            del self.handles[:]

    def GetTrajectory(self, motion):

        config_spec = self.human.GetActiveConfigurationSpecification()
        g = config_spec.GetGroupFromName('joint_values')
        g.interpolation = 'linear'
        config_spec = ConfigurationSpecification()
        config_spec.AddGroup(g)
        config_spec.AddDeltaTimeGroup()

        traj = RaveCreateTrajectory(self.human.GetEnv(), '')
        traj.Init(config_spec)
        t = 0.0
        alpha = 4  # Time scaling

        for q in motion:
            wp = self.human.GetDOFValues()
            for i, dof in enumerate(q):
                if mapping[i] >= 0:
                    wp[mapping[i]] = dof * pi / 180
            dt = q[0] - t  # q[0] is time
            wp = append(wp, alpha*dt)
            traj.Insert(traj.GetNumWaypoints(), wp)
            t = q[0]

        return traj


if __name__ == "__main__":

    h = human()
    print "Press return to play trajectory."
    sys.stdin.readline()

    while True:
        h.PlayMarkers()
        #print "Press return to exit."
        #sys.stdin.readline()

    # h.human.SetDOFValues([90 * pi / 180], [13])
    # print "Press return to play trajectory."
    # sys.stdin.readline()
    #
    # h.PlayTrajectory()
    # print "Press return to exit."
    # sys.stdin.readline()