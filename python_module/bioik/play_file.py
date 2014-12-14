#!/usr/bin/python

from openravepy import *
from numpy import *
import sys
import time
from copy import deepcopy
import csv

import rospy
from sensor_msgs.msg import JointState

class PlayFile():

    def __init__(self, environment_file):

        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.Reset()
        self.env.Load()
        self.humans = self.env.GetRobots()
        self.handles = []
        t_cam = array([[ -0.662516847837, 0.365861186797, -0.653618404214, 3.09212255478] , \
                        [ 0.748220341461, 0.282254919974, -0.600415256947, 2.43832302094] , \
                        [ -0.0351816281021, -0.886835809012, -0.46074342198, 2.15959310532] , \
                        [ 0.0, 0.0, 0.0, 1.0]])
        self.env.GetViewer().SetCamera(t_cam)

        self.traj_human1 = []
        self.traj_human2 = []

        self.change_color_human()


    def change_color_human(self):

        if len(self.humans) <= 1:
            return

        links = []
        for jIdx, j in enumerate(self.humans[1].GetJoints()):
            # print "%s, \t%.3f, \t%.3f" % (j.GetName(), j.GetLimits()[0], j.GetLimits()[1])
            l = j.GetFirstAttached()
            if l is not None : links.append(l)
            l = j.GetSecondAttached()
            if l is not None : links.append(l)

        for l in links:
            for g in l.GetGeometries():
                print g.GetDiffuseColor()
                if set(g.GetDiffuseColor()) & set([0.80000001, 0., 0.01]):
                    g.SetDiffuseColor([0., 0., 0.8])


    def print_view(self):

        print "GetViewer().GetCameraTransform() : "
        t_cam = self.env.GetViewer().GetCameraTransform()

        line_str = ""
        line_str += 'array(['
        for i in range(4):
            line_str += '['
            for j in range(4):
                line_str += ' ' + str(t_cam[i, j]) + ','
            line_str = line_str.rstrip(',')
            line_str += ']'
            if i < 3:
                line_str += ' , \\ \n'
        line_str += '])'

        print line_str


    def load_files(self, h1_filepath, h2_filepath):
        
        print "Trying to open file"

        # Parse CSV files
        with open(h1_filepath, 'r') as h1_file:
            with open(h2_filepath, 'r') as h2_file:
                self.traj_human1 = [row for row in csv.reader(h1_file, delimiter=',')]
                self.traj_human2 = [row for row in csv.reader(h2_file, delimiter=',')]

        # Convert to floats
        self.traj_human1 = [map(float, row) for row in self.traj_human1]
        self.traj_human2 = [map(float, row) for row in self.traj_human2]


    def play_skeleton(self):
        # for frame in self.frames:
        print len(self.traj_human1)

        scale = 2.

        for row1, row2 in zip(self.traj_human1, self.traj_human2):

            time.sleep(row1[0]*scale)

            del self.handles[:]
           
            self.humans[0].SetDOFValues(row1[1:self.humans[0].GetDOF()+1])
            self.humans[1].SetDOFValues(row2[1:self.humans[1].GetDOF()+1])

            if self.joint_state_pub is not None :
                self.publish_joint_state(joint_state)

    def set_publish_joint_state(self, publish_joint_state):

        if publish_joint_state:
            self.joint_state_pub = rospy.Publisher('mocap_human_joint_state', JointState)
        else:
            self.joint_state_pub = None

    def publish_joint_state(self):

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time()

        joints = self.humans[0].GetJoints()

        if len(joints) != self.humans[0].GetDOF():
            rospy.logerror("OpenRave human model is not consistant")
            return

        joint_state.name        = [""]*len(joints)
        joint_state.position    = [0.]*len(joints)

        q_cur = self.humans[0].GetDOFValues()

        for i, joint in enumerate(joints):
            joint_state.name[i] = joint.GetName()
            joint_state.position[i] = q_cur[joint.GetDOFIndex()]

        # Publish joint states
        self.joint_state_pub.publish(joint_state)

if __name__ == "__main__":

    environment_file = "../../ormodels/humans_bio_env.xml"
    h1_file = None
    h2_file = None
    publish_joint_state = rospy.get_param("~human_tracker_publish_joint_state", False)

    for index in range(1, len(sys.argv)):
        if sys.argv[index] == "-h1" and index+1 < len(sys.argv):
            h1_file = str(sys.argv[index+1]) + '/'
        if sys.argv[index] == "-h2" and index+1 < len(sys.argv):
            h2_file = int(sys.argv[index+1])
        if sys.argv[index] == "-env" and index+1 < len(sys.argv):
            environment_file = int(sys.argv[index+1])

    if h1_file is None:

        print "Usage : "
        print " -h1 /path/to/directory/file1.csv   : sets the file for human1"
        print " -h2 /path/to/directory/file1.csv   : sets the file for human2"

    else :

        print "try to load file : ", h1_file
        print "try to load file : ", h2_file

        test = PlayFile()
        test.load_files(h1_file, h2_file)
        test.set_publish_joint_state( publish_joint_state )

        while True:
            test.play_skeleton()
            test.print_view()
            print "press enter to play again"
            sys.stdin.readline()