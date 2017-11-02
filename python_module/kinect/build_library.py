#!/usr/bin/env python
# Copyright (c) 2013 Worcester Polytechnic Institute
# Author: Jim Mainrpice <jmainprice@wpi.edu>
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
import roslib;
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
import keystroke
from time import sleep
import segment_file
import shutil
from extractImages import getImages

play_folder = False
show_images = 0  # 0 to not show
m_dir = "/home/rafi/workspace/experiment/6/Run1/"
out_dir = "/home/rafi/workspace/experiment/filtered_lib/6/"
#trajectories_files = ["motion_saved_00000_00000.csv", "motion_saved_00001_00000.csv"]

class kinect_subscriber():
    def __init__(self):
        self.orEnv = Environment()
        self.prob = None
        self.h = None
        self.orEnv.SetViewer('qtcoin')

        self.currentFileOne = ''
        self.currentFileTwo = ''

        print "hello!"

        self.split = [0, 0]

        print "start"
        self.orEnv.SetDebugLevel(DebugLevel.Verbose)
        self.orEnv.Reset()

        # self.orEnv.Load("../ormodels/human_wpi.xml")
        self.orEnv.Load("../ormodels/human_wpi_new.xml")
        self.orEnv.Load("../ormodels/human_wpi_blue.xml")

        self.orEnv.Load("../ormodels/env.xml")

        print "draw frame"
        T = MakeTransform(eye(3), transpose(matrix([0, 0, 0])))
        self.h = misc.DrawAxes(self.orEnv, matrix(T), 1)

        print "try to create problem"
        self.prob = RaveCreateProblem(self.orEnv, 'Kinect')

    def play(self):
        fileList = []

        self.prob.SendCommand('SetCustomTracker ' + str(1))
        self.prob.SendCommand(
            'EnableCamera ' + str(show_images) + ' ' + m_dir + 'images/')
        self.prob.SendCommand('SetTrajectoryControl 0')

        for root, dirs, filenames in os.walk(m_dir):
            fileList = sort(filenames)
            break

        for i in xrange(0, len(fileList), 2):
            print m_dir + fileList[i]
            print m_dir + fileList[i + 1]
            self.currentFileOne = fileList[i]
            self.currentFileTwo = fileList[i + 1]
            self.prob.SendCommand('LoadTrajectoryFile ' + m_dir + fileList[i])
            self.prob.SendCommand(
                'LoadTrajectoryFile ' + m_dir + fileList[i + 1])
            self.prob.SendCommand('PlayTrajectoryFiles')

            self.keyboardControll()

    def keyboardControll(self):
        currentFrame = 0
        while True:
            #print "Enter new character"
            c = keystroke.getch(-1)
            #print c
            if c == 'q':
                self.prob.SendCommand('ResetTrajectoryFiles')
                break
            if c == 'r':
                self.prob.SendCommand('ReplayTrajectoryFiles')
            if c == '1':
                print "Marking files: " + self.currentFileOne + " and " + self.currentFileTwo + " as"
                print "----------------------GOOD--------------------------"
                outFileOne = out_dir + 'good/' + self.currentFileOne
                outFileTwo = out_dir + 'good/' + self.currentFileTwo

                # Copy the file
                shutil.copy2(m_dir + self.currentFileOne, outFileOne)
                shutil.copy2(m_dir + self.currentFileTwo, outFileTwo)
                # Move its associated images
                getImages(m_dir + self.currentFileOne, out_dir + 'images/')
                getImages(m_dir + self.currentFileTwo, out_dir + 'images/')

                #Include spaces in the name
                os.rename(outFileOne, outFileOne.replace('#', ' '))
                os.rename(outFileTwo, outFileTwo.replace('#', ' '))

                self.prob.SendCommand('ResetTrajectoryFiles')
                break
            if c == '2':
                print "Marking files: " + self.currentFileOne + " and " + self.currentFileTwo + " as"
                print "----------------------OK--------------------------"
                outFileOne = out_dir + 'ok/' + self.currentFileOne
                outFileTwo = out_dir + 'ok/' + self.currentFileTwo

                # Copy the file
                shutil.copy2(m_dir + self.currentFileOne, outFileOne)
                shutil.copy2(m_dir + self.currentFileTwo, outFileTwo)
                # Move its associated images
                getImages(m_dir + self.currentFileOne, out_dir + 'images/')
                getImages(m_dir + self.currentFileTwo, out_dir + 'images/')

                #Include spaces in the name
                os.rename(outFileOne, outFileOne.replace('#', ' '))
                os.rename(outFileTwo, outFileTwo.replace('#', ' '))

                self.prob.SendCommand('ResetTrajectoryFiles')
                break
            if c == '3':
                print "Marking files: " + self.currentFileOne + " and " + self.currentFileTwo + " as"
                print "----------------------BAD--------------------------"
                outFileOne = out_dir + 'bad/' + self.currentFileOne
                outFileTwo = out_dir + 'bad/' + self.currentFileTwo

                # Copy the file
                shutil.copy2(m_dir + self.currentFileOne, outFileOne)
                shutil.copy2(m_dir + self.currentFileTwo, outFileTwo)
                # Move its associated images
                getImages(m_dir + self.currentFileOne, out_dir + 'images/')
                getImages(m_dir + self.currentFileTwo, out_dir + 'images/')

                #Include spaces in the name
                os.rename(outFileOne, outFileOne.replace('#', ' '))
                os.rename(outFileTwo, outFileTwo.replace('#', ' '))

                self.prob.SendCommand('ResetTrajectoryFiles')
                break

            if c == 'x':
                sys.exit(0)


    def loadFiles(self, dir, files):
        for file in files:
            self.prob.SendCommand('LoadTrajectoryFile ' + dir + file)


if __name__ == "__main__":
    print "main function"
    k = kinect_subscriber()
    print "finish init"
    k.play()
