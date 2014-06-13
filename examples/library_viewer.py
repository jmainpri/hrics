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
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from wiimote.msg import*
import keystroke
from time import sleep
import segment_file

#in order to use the wiimote, create a wiimote subscriber object and call run.

class LibraryViewer:
    def __init__(self, list_one, list_two, img_dir):
        self.orEnv = Environment()
        self.prob = None
        self.h = None
        self.orEnv.SetViewer('qtcoin')
        self.nb_human = 0
        self.human_one = list_one
        self.human_two = list_two
        self.img_dir = img_dir
        self.show_images = False

        print "start"
        self.orEnv.SetDebugLevel(DebugLevel.Verbose)
        self.orEnv.Reset()

        print "find number of humans:"
        if len(list_one) == len(list_two):
            nb_human = 2
        elif (len(list_one) > 0 and len(list_two) == 0):
            nb_human = 1
        else:
            print "Couldn't determine # of humans with : " + str(len(self.human_one)) + " and " + str(len(self.human_two)) + " motions"
            sys.exit(0)

        print "load environment"
        self.orEnv.Load("../ormodels/human_wpi_new.xml")
        if nb_human > 1 :
            self.orEnv.Load("../ormodels/human_wpi_blue.xml")

        self.orEnv.Load("../ormodels/env.xml")

        print "draw frame"
        T = MakeTransform( eye(3), transpose(matrix([0,0,0])))
        self.h = misc.DrawAxes( self.orEnv, matrix(T), 1 )

        print "try to create problem"
        self.prob = RaveCreateProblem(self.orEnv,'Kinect')

    def play(self, type ):
        print "loading files"

        self.prob.SendCommand('SetCustomTracker ' + str(self.nb_human-1) )
        if self.img_dir:
            self.show_images = True

        self.prob.SendCommand('EnableCamera ' + str(int(self.show_images)) + ' ' + self.img_dir )
        self.prob.SendCommand('SetPlayType ' + str(type))

        for i in range(0,len(self.human_one)):
            print self.human_one[i]
            print self.human_two[i]

            self.prob.SendCommand( 'LoadTrajectoryFile '+self.human_one[i] )
            self.prob.SendCommand( 'LoadTrajectoryFile '+ self.human_two[i] )
            self.prob.SendCommand('PlayTrajectoryFiles')

            self.keyboardControll()

    def keyboardControll(self):
        currentFrame = 0
        while True:
            #print "Enter new character"
            c = keystroke.getch(-1)

            if c == 'q':
                self.prob.SendCommand('ResetTrajectoryFiles')
                break
            if c == 'r':
                self.prob.SendCommand('ReplayTrajectoryFiles')
            #print c
            if c == 'x':
                sys.exit()
            if c == 'u':
                self.prob.SendCommand('ControlTrajectoryPlayback -25')
            if c == 'i':
                self.prob.SendCommand('ControlTrajectoryPlayback -1')
            if c == 'o':
                self.prob.SendCommand('ControlTrajectoryPlayback 1')
            if c == 'p':
                self.prob.SendCommand('ControlTrajectoryPlayback 25')


    def loadFiles(self, dir, files):
        for file in files:
            self.prob.SendCommand( 'LoadTrajectoryFile '+ dir + file )


if __name__ == "__main__":
    # Generate two lists of files
    list_one = []
    list_two = []

    for (dirname, dirs, files) in os.walk('/home/rafi/workspace/experiment/'):
        for filename in files:
            if filename.endswith('.csv') and filename.startswith('[') and not "filtered_lib" in dirname :
                if ('1' in filename.split('_')[2]):
                    list_two.append(os.path.join(dirname,filename))
                else:
                    list_one.append(os.path.join(dirname,filename))

    list_one.sort()
    list_two.sort()

    print "main function"
    v = LibraryViewer(list_one, list_two, "/home/rafi/workspace/experiment/filtered_lib/images/")
    v.play(2)

    print "Press return to run "
    sys.stdin.readline()

