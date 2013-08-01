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

#Tesing
import subprocess

#print "HOME : " + os.environ['HOME']
#print "PYTHONPATH : " + os.environ['PYTHONPATH']
#print "OPENRAVE_PLUGINS : " + os.environ['OPENRAVE_PLUGINS']

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

        print "start"
        self.orEnv.SetDebugLevel(DebugLevel.Verbose)
        self.orEnv.Reset()
        self.orEnv.Load("../ormodels/human_wpi.xml")
        #self.orEnv.Load("../ormodels/human_wpi_blue.xml")
        self.orEnv.Load("../ormodels/env.xml")
        
#       self.orEnv.Load("../ormodels/table.xml")

        #for robot in self.orEnv.GetRobots():
        #    print robot.GetName()
        #table = self.orEnv.GetKinBody('table')
        #table.SetTransform([0,0,0,1,1.72,0,0.73])

        print "draw frame"
        T = MakeTransform( eye(3), transpose(matrix([0,0,0]))) 
        self.h = misc.DrawAxes( self.orEnv, matrix(T), 1 )

        print "try to create problem"
        self.prob = RaveCreateProblem(self.orEnv,'Kinect')

    def listen(self):
        print "Trying to listen"
        self.prob.SendCommand('SetCustomTracker 1')
        self.prob.SendCommand('SetNumKinect 1') #still need to call as 1 if using default tracker.
        self.prob.SendCommand('EnableCamera 1')
        print "Trying to set kinect frame"
        #self.prob.SendCommand('SetKinectFrame 0 0.0 -0.2 1.6 -30.0 0.0')
        #self.prob.SendCommand('SetKinectFrame 1 0.0 0.2 1.6 30.0 0.0')

        self.prob.SendCommand('SetKinectFrame 0 0.0 0.13 1.37 35.0 0.0')
        #self.prob.SendCommand('SetKinectFrame 1 0.0 -0.13 1.37 -35.0 0.0')        

        
        self.prob.SendCommand('StartListening')

    def play(self, controlled):
        print "loading files"
        dir = "/home/rafihayne/workspace/statFiles/recorded_motion/"
        files = ["motion_saved_00000_00000.csv",
                 
                 ]   
        self.loadFiles(dir, files)

        self.prob.SendCommand('EnableCamera 1')

        if controlled:
            self.prob.SendCommand('SetTrajectoryControl 1')
        else:
            self.prob.SendCommand('SetTrajectoryControl 0')

        self.prob.SendCommand('PlayTrajectoryFiles')

        if controlled:
            sleep(1)
            print "\n \n"
            print "Controlls:     u   i    o   p      q         space"
            print "              <<<  <<  >>  >>>    exit    current frame" 
            print "Enter character:"
            self.keyboardControll()

    def keyboardControll(self):
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
                self.prob.SendCommand('GetPlaybackFrame')
        
    def loadFiles(self, dir, files):
        for file in files:
            self.prob.SendCommand( 'LoadTrajectoryFile '+ dir + file )

    def rec(self, state):
        def terminate_process_and_children(p): #Found on ros answers.
            ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
            ps_output = ps_command.stdout.read()
            retcode = ps_command.wait()
            assert retcode == 0, "ps command returned %d" % retcode
            for pid_str in ps_output.split("\n")[:-1]:
                os.kill(int(pid_str), subprocess.signal.SIGINT)
            p.terminate()

        if state:
            print "Recording Motion"
            self.prob.SendCommand('SetButtonState 1')
            #self.proc = subprocess.Popen("rosbag record -O kindata /tf", stdin=subprocess.PIPE, shell=True, cwd="/home/rafihayne/workspace/statFiles/bagfiles")

        else:
            self.prob.SendCommand('SetButtonState 0')
            #terminate_process_and_children(self.proc)
            

if __name__ == "__main__":
    print "main function"
    k = kinect_subscriber()
    k.listen()

    w = wiimote_subscriber(k.prob)
    w.run()

