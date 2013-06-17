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
import time



orEnv = Environment()
h = 0
    
def loadFiles(prob):
    dir = "/home/rafihayne/workspace/statFiles/recorded_motion/"

    files = ["test.csv",
             ]  

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
       
    print "try to create problem"
    prob = RaveCreateProblem(orEnv,'Kinect')
    
    print "tring to resample files"
    loadFiles(prob)
    cmdout = prob.SendCommand( 'ResampleFiles 100' )

    sys.exit()

    
if __name__ == "__main__":
    print "main function"
    launch()
