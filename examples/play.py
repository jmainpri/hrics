#!/usr/bin/env python

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
import keystroke
import time

#print "HOME : " + os.environ['HOME']
#print "PYTHONPATH : " + os.environ['PYTHONPATH']
#print "OPENRAVE_PLUGINS : " + os.environ['OPENRAVE_PLUGINS']

orEnv = Environment()
h = 0

def keyboardControll(prob):
    while True:
        #print "Enter new character"
        c = keystroke.getch(-1)
        #print c
        if c == 'q': 
            sys.exit()    		
        if c == 'u':
            prob.SendCommand('ControlTrajectoryPlayback -25')
        if c == 'i':
            prob.SendCommand('ControlTrajectoryPlayback -1')
        if c == 'o':
            prob.SendCommand('ControlTrajectoryPlayback 1')
        if c == 'p':
            prob.SendCommand('ControlTrajectoryPlayback 25')
        if c == ' ':
            prob.SendCommand('GetPlaybackFrame')
    
def loadFiles(prob):
    dir = "/home/rafihayne/statFiles/recorded_motion/"
    files = ["motion_saved_00000_00000.csv",
             "motion_saved_00001_00000.csv",]   


    for file in files:
        cmdout = prob.SendCommand( 'LoadTrajectoryFile '+ dir + file )

def launch():
    global orEnv
    global h
    
    orEnv.SetViewer('qtcoin')
    
    print "start"
    orEnv.SetDebugLevel(DebugLevel.Verbose)
    orEnv.Reset()
    orEnv.Load("../ormodels/human_wpi.xml")
    orEnv.Load("../ormodels/human_wpi_blue.xml")
    
    print "draw frame"
    T = MakeTransform( eye(3), transpose(matrix([0,0,0]))) 
    h = misc.DrawAxes( orEnv, matrix(T), 1 )
    
    print "try to create problem"
    prob = RaveCreateProblem(orEnv,'Kinect')
    
    print "tring to load files"
    #cmdout = prob.SendCommand('LoadTrajectoryFile /home/rafihayne/statFiles/recorded_motion/motion_saved_00000_00000.csv')
    loadFiles(prob)
    
    print "trying to play files"
    cmdout = prob.SendCommand('SetTrajectoryControl 1') #1 for controlled, 0 for normal playback
    cmdout = prob.SendCommand('PlayTrajectoryFiles')


    time.sleep(1)
    print "Enter character"
    keyboardControll(prob)
    	
    	
    sys.stdin.readline()

    
if __name__ == "__main__":
    print "main function"
    launch()
