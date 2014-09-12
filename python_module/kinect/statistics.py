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
import roslib;
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from wiimote.msg import*
import keystroke
from time import sleep
import segment_file
#/home/rafi/workspace/experiment/2/Run0/[1016#-#1112]#motion_saved_00000_00000.csv
trajectories_directory = "/home/rafi/workspace/experiment/2/Run0/"
trajectories_files = ["[1016#-#1112]#motion_saved_00000_00000.csv", "[1016#-#1112]#motion_saved_00001_00000.csv"]

#in order to use the wiimote, create a wiimote subscriber object and call run.

class kinect_subscriber():
    def __init__(self, list_one, list_two):
        self.orEnv = Environment()
        self.prob = None
        self.h = None
        self.human_one = list_one
        self.human_two = list_two
        self.dir = trajectories_directory
        self.files = trajectories_files


        print "start"
        self.orEnv.SetDebugLevel(DebugLevel.Verbose)
        self.orEnv.Reset()

        self.orEnv.Load("../ormodels/human_wpi_new.xml")
        if len(self.files) > 1 :
            self.orEnv.Load("../ormodels/human_wpi_blue.xml")


        print "try to create problem"
        self.prob = RaveCreateProblem(self.orEnv,'Kinect')

        print "try to init move3d"
        self.prob.SendCommand('InitMove3D')

    def play(self ):
            self.prob.SendCommand('SetPlayType 2')

            for i in range(len(self.human_one)):
                self.prob.SendCommand( 'LoadTrajectoryFile '+self.human_one[i] )
                self.prob.SendCommand( 'LoadTrajectoryFile '+ self.human_two[i] )
                self.prob.SendCommand('PlayTrajectoryFiles')
                sleep(1)
                self.prob.SendCommand('ResetTrajectoryFiles')


    def loadFiles(self, dir, files):
        for file in files:
            self.prob.SendCommand( 'LoadTrajectoryFile '+ dir + file )


if __name__ == "__main__":
    print "main function"
    list_one = []
    list_two = []

    for (dirname, dirs, files) in os.walk('/home/rafi/workspace/experiment/'):
        for filename in files:
            if filename.endswith('.csv') and filename.startswith('[') and "replan" in dirname :
                if ('human_one' in dirname):
                    list_one.append(os.path.join(dirname,filename))
                else:
                    list_two.append(os.path.join(dirname,filename))

    list_one.sort()
    list_two.sort()

    k = kinect_subscriber(list_one, list_two)
    k.play()

    for i, file in enumerate(list_one):
        print str(i) + " : " + file

    print "Press return to run "
    sys.stdin.readline()


