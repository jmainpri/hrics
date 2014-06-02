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

play_folder = False
show_images = 1 # 0 to not show
base_dir = "/home/rafi/workspace/experiment/1/"
#trajectories_files = ["motion_saved_00000_00000.csv", "motion_saved_00001_00000.csv"]

class kinect_subscriber():
    def __init__(self):
        self.orEnv = Environment()
        self.prob = None
        self.h = None
        self.orEnv.SetViewer('qtcoin')

        print "hello!"

        self.split = [0,0]

        print "start"
        self.orEnv.SetDebugLevel(DebugLevel.Verbose)
        self.orEnv.Reset()

        # self.orEnv.Load("../ormodels/human_wpi.xml")
        self.orEnv.Load("../ormodels/human_wpi_new.xml")
        self.orEnv.Load("../ormodels/human_wpi_blue.xml")

        self.orEnv.Load("../ormodels/env.xml")

        print "draw frame"
        T = MakeTransform( eye(3), transpose(matrix([0,0,0])))
        self.h = misc.DrawAxes( self.orEnv, matrix(T), 1 )

        print "try to create problem"
        self.prob = RaveCreateProblem(self.orEnv,'Kinect')

    def play(self):
        fileList = []

        self.prob.SendCommand('SetCustomTracker ' + str(1) )
        self.prob.SendCommand('EnableCamera ' + str(show_images) + ' ' + base_dir + 'images/' )
        self.prob.SendCommand('SetTrajectoryControl 0')

        for root, dirs, filenames in os.walk(base_dir+'Run0/'):
            fileList = sort(filenames)
            break

        print len(fileList)
        for i in xrange(0,len(fileList), 2):
            self.prob.SendCommand('ResetTrajectoryFiles')
            print base_dir+'' + fileList[i]
            print base_dir+'' + fileList[i+1]
            self.prob.SendCommand( 'LoadTrajectoryFile '+ base_dir+'Run0/' + fileList[i] )
            self.prob.SendCommand( 'LoadTrajectoryFile '+ base_dir+'Run0/' + fileList[i+1] )
            self.prob.SendCommand('PlayTrajectoryFiles')

            self.keyboardControll()

    def keyboardControll(self):
        currentFrame = 0
        while True:
            #print "Enter new character"
            c = keystroke.getch(-1)
            #print c
            if c == 'q':
                break
            if c == 'u':
                self.prob.SendCommand('ControlTrajectoryPlayback -25')
            if c == 'i':
                self.prob.SendCommand('ControlTrajectoryPlayback -1')
            if c == 'o':
                self.prob.SendCommand('ControlTrajectoryPlayback 1')
            if c == 'p':
                self.prob.SendCommand('ControlTrajectoryPlayback 25')
            if c == ' ':
                currentFrame = int(self.prob.SendCommand('GetPlaybackFrame'))
                print "Current Frame: " + str(currentFrame)
            if c == '1':
                currentFrame = int(self.prob.SendCommand('GetPlaybackFrame'))
                self.split[0] = currentFrame
                print "Set split beginning to: " + str(self.split[0])
            if c == '2':
                currentFrame = int(self.prob.SendCommand('GetPlaybackFrame'))
                self.split[1] = currentFrame
                print "Set split ending to: " + str(self.split[1])
            if c == 's':
                for file in self.files:
                    print "Segmenting file: " + file + " from: " + str(self.split[0]) + " to: " + str(self.split[1])
                    segment_file.segment([(self.split[0], self.split[1])], self.dir+file)


    def loadFiles(self, dir, files):
        for file in files:
            self.prob.SendCommand( 'LoadTrajectoryFile '+ dir + file )


if __name__ == "__main__":
    print "main function"
    k = kinect_subscriber()
    print "finish init"
    k.play()


