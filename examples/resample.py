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
    dir = "/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/"

    files = ["motion_saved_00000_00063.csv",
"motion_saved_00000_00087.csv",
"motion_saved_00000_00143.csv",
"motion_saved_00000_00191.csv",
"motion_saved_00000_00079.csv",
"motion_saved_00000_00199.csv",
"motion_saved_00000_00167.csv",
"motion_saved_00000_00135.csv",
"motion_saved_00000_00183.csv",
"motion_saved_00000_00175.csv",
"motion_saved_00000_00023.csv",
"motion_saved_00000_00151.csv",
"motion_saved_00000_00031.csv",
"motion_saved_00000_00159.csv",
"motion_saved_00000_00015.csv",
"motion_saved_00000_00055.csv",
"motion_saved_00000_00007.csv",
"motion_saved_00000_00047.csv",
"motion_saved_00000_00119.csv",
"motion_saved_00000_00127.csv",
"motion_saved_00000_00071.csv",
"motion_saved_00000_00111.csv",
"motion_saved_00000_00103.csv",
"motion_saved_00000_00095.csv",
"motion_saved_00000_00039.csv",




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
