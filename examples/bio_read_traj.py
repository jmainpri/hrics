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
from numpy import *
from TransformMatrix import *
from rodrigues import *
import roslib; roslib.load_manifest('wiimote')
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from wiimote.msg import*
import keystroke
from time import sleep
import segment_file

mapping = [-1, 6, 7, 8, 12, 13, 14, 17, 18, 19]

class human():

    def __init__(self):
        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.Reset()
        self.env.Load("../ormodels/human_wpi_bio.xml")
        # self.env.Load("../ormodels/human_wpi_new.xml")
        # self.orEnv.Load("robots/pr2-beta-static.zae")

        motion = numpy.genfromtxt('output.ik.csv', delimiter=',')
        motion = numpy.delete(motion, 0, axis=0)
        # print motion.shape
        for row in motion:
            print ' '.join(map(str, row))

        # self.prob = RaveCreateProblem(self.env, 'Kinect')
         #self.prob.SendCommand('InitMove3D')

        self.human = self.env.GetRobots()[0]
        self.handles = []

        for j in self.human.GetJoints():
            t_link = j.GetHierarchyChildLink().GetTransform()
            self.handles.append(misc.DrawAxes(self.env, t_link, 0.3))

        traj = self.GetHumanTrajectory(motion)
        self.human.GetController().SetPath(traj)
        self.human.WaitForController(0)

    # def SetConfiguration(row):

    def GetHumanTrajectory(self, motion):

        configSpec = self.human.GetActiveConfigurationSpecification()
        g = configSpec.GetGroupFromName('joint_values')
        g.interpolation = 'linear'
        configSpec = ConfigurationSpecification()
        configSpec.AddGroup(g)
        # Uncomment for velocities
        # configSpec.AddDerivativeGroups(1,False)
        configSpec.AddDeltaTimeGroup()

        traj = RaveCreateTrajectory(self.human.GetEnv(), '')
        traj.Init(configSpec)
        t = 0.0
        dt = 0.001

        for q in motion:
            wp = self.human.GetDOFValues()
            for i, dof in enumerate(q):
                if mapping[i] >= 0:
                    wp[mapping[i]] = dof * pi / 180
            wp = append(wp,  t)
            traj.Insert(traj.GetNumWaypoints(), wp)
            t += dt

        return traj


if __name__ == "__main__":
    print "main function"
    k = human()
    print "Press return to exit."
    sys.stdin.readline()


