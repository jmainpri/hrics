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
import roslib; #roslib.load_manifest('wiimote')
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from time import sleep
#import segment_file


class human():
    def __init__(self):
        self.orEnv = Environment()
        self.orEnv.SetViewer('qtcoin')
        self.orEnv.SetDebugLevel(DebugLevel.Verbose)
        self.orEnv.Reset()
        self.orEnv.Load("../ormodels/human_wpi.xml")
        # self.orEnv.Load("robots/pr2-beta-static.zae")

        # self.prob = RaveCreateProblem(self.orEnv,'Kinect')
        # self.prob.SendCommand('InitMove3D')

if __name__ == "__main__":
    print "main function"
    k = human()
    print "Press return to exit."
    sys.stdin.readline()


