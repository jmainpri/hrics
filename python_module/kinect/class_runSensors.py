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
#trajectories_directory = "/home/rafi/workspace/statFiles/recorded_motion/"
#trajectories_directory = "/home/rafi/Desktop/classes/"
#trajectories_directory = "/media/57f621de-c63b-4d30-84fc-da4ce0b1e1eb/home/rafihayne/workspace/statFiles/saved/8/"
trajectories_directory = "/home/rafi/workspace/experiment/6/"
#trajectories_files = ["temp.csv"] #One file for each human in the scene
trajectories_files = ["motion_saved_00000_00000.csv", "motion_saved_00001_00000.csv"]

#in order to use the wiimote, create a wiimote subscriber object and call run.

class wiimote_subscriber():
    def __init__(self, a_prob):
        self.record_state = False
        self.is_pressed = False
        self.a_prob = a_prob

    def callback(self, State):
        if State.buttons[4] == True and self.is_pressed == False:
            self.record_state = False
            self.is_pressed = True
            cmdout = self.a_prob.SendCommand('SetButtonState 1')

        if State.buttons[4] == False and self.is_pressed == True:
            self.is_pressed = False
            cmdout = self.a_prob.SendCommand('SetButtonState 0')

    def run(self):
        rospy.init_node('wii_listener')
        rospy.Subscriber("/wiimote/state", State, self.callback)
        rospy.spin()

class kinect_subscriber():
    def __init__(self):
        self.orEnv = Environment()
        self.prob = None
        self.h = None
        self.orEnv.SetViewer('qtcoin')

        self.dir = trajectories_directory
        self.files = trajectories_files

        self.split = [0,0]

        print "start"
        self.orEnv.SetDebugLevel(DebugLevel.Verbose)
        self.orEnv.Reset()

        # self.orEnv.Load("../ormodels/human_wpi.xml")
        self.orEnv.Load("../ormodels/human_wpi_new.xml")
        if len(self.files) > 1 :
            self.orEnv.Load("../ormodels/human_wpi_blue.xml")

        #self.orEnv.Load("../ormodels/env.xml")

        T_h = MakeTransform( eye(3), transpose(matrix([0,0,-10])))
        human1 = self.orEnv.GetRobots()[0].SetTransform(array(T_h))
        human2 = self.orEnv.GetRobots()[1].SetTransform(array(T_h))

        print "draw frame"
        T = MakeTransform( eye(3), transpose(matrix([0,0,0])))

        self.h = misc.DrawAxes( self.orEnv, matrix(T), 1 )

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
        #self.prob.SendCommand('SetKinectFrame 0 -0.3556 -0.8636 1.3208 62.0 7.0')
        #self.prob.SendCommand('SetKinectFrame 1 1.1938 0.7493 1.2446 -125 0.0') 
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

        sleep(2)
        #self.prob.SendCommand('DrawMocapFile /home/rafi/workspace/hrics-or-plugins/examples/positions_fixed.csv')
        self.prob.SendCommand('DrawMocapFile /home/rafi/workspace/hrics-or-plugins/examples/markers_fixed.csv')
        #self.prob.SendCommand('DrawMocapFile //home/rafi/catkin_ws/src/vicon_logger/positions.csv')
        return

        if play_folder :
            self.prob.SendCommand('PlayTrajectoryFolder /home/rafi/workspace/experiment/1/Run0/')
        else :
            self.prob.SendCommand('PlayTrajectoryFiles')

        if controlled:
            sleep(1)
            print "\n \n"
            print "Controlls:     u   i    o   p      q         space            1            2          s"
            print "              <<<  <<  >>  >>>    exit    current frame  split start  split end    segment"
            print "Enter character:"
            self.keyboardControll()

    def keyboardControll(self):
        currentFrame = 0
        while True:
            #print "Enter new character"
            c = keystroke.getch(-1)
            #print c
            if c == 'q':
                sys.exit()
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
    #    k.listen()

    #    w = wiimote_subscriber(k.prob)
    #    w.run()

    print "Press return to run "
    sys.stdin.readline()
    

