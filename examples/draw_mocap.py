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
import transformation_helper

play_folder = False
show_images = 1 # 0 to not show
#trajectories_directory = "/home/rafi/workspace/statFiles/recorded_motion/"
#trajectories_directory = "/home/rafi/Desktop/classes/"
#trajectories_directory = "/media/57f621de-c63b-4d30-84fc-da4ce0b1e1eb/home/rafihayne/workspace/statFiles/saved/8/"
trajectories_directory = "/home/rafi/workspace/experiment/6/"
#trajectories_files = ["temp.csv"] #One file for each human in the scene
trajectories_files = ["motion_saved_00000_00000.csv", "motion_saved_00001_00000.csv"]

#in order to use the wiimote, create a wiimote subscriber object and call run.

class kinect_subscriber():
    def __init__(self):
        self.orEnv = Environment()
        self.prob = None
        self.h = [None]
        self.orEnv.SetViewer('qtcoin')

        self.dir = trajectories_directory
        self.files = trajectories_files

        self.split = [0,0]

        print "start"
        self.orEnv.SetDebugLevel(DebugLevel.Verbose)
        self.orEnv.Reset()

        self.orEnv.Load("../ormodels/human_wpi_new.xml")
        if len(self.files) > 1 :
            self.orEnv.Load("../ormodels/human_wpi_blue.xml")

        #self.orEnv.Load("../ormodels/env.xml")

        T_h = MakeTransform( eye(3), transpose(matrix([0,0,-10])))
        human1 = self.orEnv.GetRobots()[0].SetTransform(array(T_h))
        human2 = self.orEnv.GetRobots()[1].SetTransform(array(T_h))

        print "draw frame"
        T = MakeTransform( eye(3), transpose(matrix([0,0,0])))

        # Second Run
        # CalBlock = MakeTransform( rotationMatrixFromQuat(array([0.7188404833, 0.2970505392, -0.4412091838, 0.4476201435 ])), transpose(matrix([1.8211836689, 1.0781754454, 1.7937961156])))
        # TouchTomorrow3 =  MakeTransform( rotationMatrixFromQuat(array([0.1457896082, -0.4695454709, 0.8700436823, 0.0360059973])), transpose(matrix([1.9327413784, 1.3481940541, 1.0716016861])))


        # Third Run
        ArchieLeftHand = MakeTransform( rotationMatrixFromQuat( array(transformation_helper.NormalizeQuaternion([-0.0565983288, 0.6551985404, -0.0308211559, 0.7527028352]) )), transpose(matrix([ 2.2630624073, 1.5024087108, 1.0334129255 ])) )
        ArchieRightHand = MakeTransform( rotationMatrixFromQuat( array(transformation_helper.NormalizeQuaternion([0.8364161312, -0.0441252319, -0.5430014612, -0.0600868744])) ), transpose(matrix([2.0138498402, 1.5327076092, 1.7597781595])) )
        CalBlock = MakeTransform( rotationMatrixFromQuat( array(transformation_helper.NormalizeQuaternion([0.7334740645, -0.2377743781, -0.4547046476, -0.4457833838]) )), transpose(matrix([1.3887227848, 0.894679857, 1.7826058767])) )
        TouchTomorrow3 = MakeTransform( rotationMatrixFromQuat( array(transformation_helper.NormalizeQuaternion([0.0351806021, 0.7399746771, 0.6593515387, -0.1282784114]) )), transpose(matrix([1.4964491222, 0.6294067075, 1.046128491])) )



        self.h.append(misc.DrawAxes( self.orEnv, matrix(T), 1 ))
        # H1 Head
        # self.h.append(misc.DrawAxes( self.orEnv, matrix(CalBlock), 1 ))
        # H1 Pelv
        # self.h.append(misc.DrawAxes( self.orEnv, matrix(TouchTomorrow3), 1 ))
        # H2 Pelv
        # self.h.append(misc.DrawAxes( self.orEnv, matrix(ArchieLeftHand), 1 ))
        # H2 Head
        # self.h.append(misc.DrawAxes( self.orEnv, matrix(ArchieRightHand), 1 ))

        print "try to create problem"
        self.prob = RaveCreateProblem(self.orEnv,'Kinect')

        print "try to init move3d"
        self.prob.SendCommand('InitMove3D')

    def listen(self):
        print "Trying to listen"
        self.prob.SendCommand('SetCustomTracker 0')
        self.prob.SendCommand('SetNumKinect 1')  # still need to call as 1 if using default tracker.
        self.prob.SendCommand('EnableCamera ' + str(show_images))

        print "Trying to set kinect frame"
        #Dual Kinect Across Setup.
        self.prob.SendCommand('UsePR2Frame 0')

        #Single Kinect
        self.prob.SendCommand('SetKinectFrame 0 0.49 -1.02 1.32 90.0 7.0')

        print "Python: starting listener!"
        self.prob.SendCommand('StartListening')

    def play(self, controlled, play_folder=False ):
        print "loading files"

        if not play_folder :
            self.loadFiles(self.dir, self.files)

        #self.prob.SendCommand('SetCustomTracker ' + str(len(self.files)-1) ) #FIX THIS ASAP.  MESSY kin prob enable camera
        self.prob.SendCommand('SetCustomTracker 1');
        self.prob.SendCommand('EnableCamera ' + str(show_images) + ' ' + trajectories_directory + 'images/' )

        self.prob.SendCommand('SetPlayType 1')

        self.prob.SendCommand('DrawMocapFile /home/rafi/workspace/hrics-or-plugins/examples/markers_fixed.csv /home/rafi/logging_data/third/objects.csv')
        # self.prob.SendCommand('DrawMocapFile /home/rafi/logging_data/third/markers.csv /home/rafi/logging_data/third/objects.csv')

        return

    def loadFiles(self, dir, files):
        for file in files:
            file.replace(' ', '\\ ')
            print "Trying to load " + dir + file
            self.prob.SendCommand( 'LoadTrajectoryFile '+ dir + file )

    def rec(self, state):
        if state:
            print "Recording Motion"
            self.prob.SendCommand('SetButtonState 1')

        else:
            self.prob.SendCommand('SetButtonState 0')


if __name__ == "__main__":
    print "main function"
    k = kinect_subscriber()
    print "press return to run"
    sys.stdin.readline()
    k.play(1)

    print "Press return to exit "
    sys.stdin.readline()
    

