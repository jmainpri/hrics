#!/usr/bin/python

from openravepy import *
from numpy import *
import sys
import time
from copy import deepcopy
import csv

mapping = [-1, 6, 7, 8, 16, 17, 18, 20, 21, 22, 24, 25, 26]

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

        t_cam = array([[ -0.655253290114, -0.106306078558, 0.747891799297, -0.302201271057] , \
                        [ -0.725788890663, 0.363116971923, -0.584274379801, 2.68592453003] , \
                        [ -0.209460287369, -0.925659269042, -0.315089361376, 2.25037527084] , \
                        [ 0.0, 0.0, 0.0, 1.0]])
        self.env.GetViewer().SetCamera(t_cam)

    def PrintView(self):

        while True:

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
            print "Press return to get view matrix."
            sys.stdin.readline()

    def load_files(self, h1_filepath, h2_filepath):
        print "Trying to open file"

        with open(h1_filepath, 'r') as h1_file:
            with open(h2_filepath, 'r') as h2_file:

                self.traj_human1 = [row for row in csv.reader(h1_file, delimiter=',')]
                self.traj_human2 = [row for row in csv.reader(h2_file, delimiter=',')]

        traj1_tmp = []
        traj2_tmp = []
        for row1, row2 in zip(self.traj_human1, self.traj_human2):
            row1_tmp = []
            row2_tmp = []
            for cell1, cell2 in zip(row1,row2):
                row1_tmp.append( float(cell1) )
                row2_tmp.append( float(cell2) )
            traj1_tmp.append(row1_tmp)
            traj2_tmp.append(row2_tmp)

        self.traj_human1 = traj1_tmp
        self.traj_human2 = traj2_tmp
		

    def play_skeleton(self):
        # for frame in self.frames:
        print len(self.traj_human1)
        prev_time = self.traj_human1[0][0]

        for row1, row2 in zip(self.traj_human1, self.traj_human2):

            curr_time = row1[0]
            dt = curr_time - prev_time
            prev_time = curr_time
            print dt
            time.sleep(0.05)

            del self.handles[:]
           
            self.humans[0].SetDOFValues(row1[1:self.humans[0].GetDOF()+1])
            self.humans[1].SetDOFValues(row2[1:self.humans[1].GetDOF()+1])

            # if i % 3 == 0:
            #     print "press enter to continue"
            #     sys.stdin.readline()
            for h in self.humans:
                print "robot in collision ", h.CheckSelfCollision()

    def run(self):

        self.play_skeleton()

if __name__ == "__main__":

    folder = '/home/ruikun/workspace/gmm-gmr-gesture-recognition/data_ten_motions_mocap_1016/'

    name = '[0408-0491]'
    h1_file = folder + "human_one/" + name + '_human1_.csv'
    h2_file = folder + "human_two/" + name + '_human2_.csv'

    print "try to load file : ", h1_file
    print "try to load file : ", h2_file

    test = PlayFile()

    while True:
        test.load_files(h1_file, h2_file)
        test.play_skeleton()
        print "press enter to exit"
        sys.stdin.readline()
