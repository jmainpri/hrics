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
from os import listdir
from os.path import isfile, join


orEnv = Environment()
h = 0
    
def loadFiles(prob):
    dir = "/home/rafi/Desktop/Library/"

    for i in range(8):
        for j in range(25):
            cmdout = prob.SendCommand( 'LoadTrajectoryFile '+ dir + "motion_saved_00000_" + str("{0:05d}".format( (j*8)+i )) +".csv")

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
