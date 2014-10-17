#!/usr/bin/python

from openravepy import *
from numpy import *
import sys
import time
from copy import deepcopy
import csv

class PlayFile():

    def __init__(self):

        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.Reset()
        self.env.Load("../../ormodels/humans_bio_env.xml")
        self.humans = self.env.GetRobots()
        self.handles = []

        self.traj_human1 = []
        self.traj_human2 = []
        self.offset_pelvis_torso_init = self.humans[0].GetJoint("TorsoX").GetHierarchyChildLink().GetTransform()[0:3, 3]

        t_cam = array([[ -0.662516847837, 0.365861186797, -0.653618404214, 3.09212255478] , \
                        [ 0.748220341461, 0.282254919974, -0.600415256947, 2.43832302094] , \
                        [ -0.0351816281021, -0.886835809012, -0.46074342198, 2.15959310532] , \
                        [ 0.0, 0.0, 0.0, 1.0]])
        self.env.GetViewer().SetCamera(t_cam)


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

    def run(self):

        self.play_skeleton()

if __name__ == "__main__":

    # folder = '/home/ruikun/workspace/gmm-gmr-gesture-recognition/data_ten_motions_mocap_1016/'
    folder = 'ten_motions/'

    name = '[0408-0491]'
    h1_file = folder + "human_one/" + name + '_human1_.csv'
    h2_file = folder + "human_two/" + name + '_human2_.csv'

    print "try to load file : ", h1_file
    print "try to load file : ", h2_file

    test = PlayFile()
    test.load_files(h1_file, h2_file)

    while True:
        test.play_skeleton()
        test.print_view()
        print "press enter to exit"
        sys.stdin.readline()
