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
		

def launch():

    
    
    global orEnv
    global h
    
    orEnv.SetViewer('qtcoin')
    
    print "start"
    orEnv.SetDebugLevel(DebugLevel.Verbose)
    orEnv.Reset()
    orEnv.Load("../ormodels/human_wpi.xml")
    #orEnv.Load("../ormodels/human_wpi_blue.xml")
    
    print "draw frame"
    T = MakeTransform( eye(3), transpose(matrix([0,0,0]))) 
    h = misc.DrawAxes( orEnv, matrix(T), 1 )
    
    print "try to create problem"
    prob = RaveCreateProblem(orEnv,'Kinect')
    
    print "tring to load files"
    #cmdout = prob.SendCommand('LoadTrajectoryFile /home/rafihayne/statFiles/recorded_motion/motion_saved_00000_00000.csv')
    loadFiles(prob)
    
    print "trying to play files"
    cmdout = prob.SendCommand('PlayTrajectoryFiles')


    time.sleep(5)
    print "Enter character"
    
    while True:
    	print "Enter new character"
    	c = keystroke.getch(-1)
    	print c
    	if c == 'q': 
    		sys.exit()
	if c == 'i':
		print "backward"
	if c == 'o':
		print "forward"
    	
    	
    sys.stdin.readline()

    
def loadFiles(prob):
	files = ["motion_saved_00000_00000.csv"]
	dir = "/home/rafihayne/statFiles/recorded_motion/"
	
	for i in files:
		cmdout = prob.SendCommand( 'LoadTrajectoryFile '+ dir +i )
	
    
if __name__ == "__main__":
    print "main function"
    launch()
