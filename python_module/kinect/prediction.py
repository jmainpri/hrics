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
from TransformMatrix import *
from rodrigues import *
import roslib;
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from time import sleep


#print "HOME : " + os.environ['HOME']
#print "PYTHONPATH : " + os.environ['PYTHONPATH']
#print "OPENRAVE_PLUGINS : " + os.environ['OPENRAVE_PLUGINS']

class prediction_subscriber():
    def __init__(self):
        self.orEnv = Environment()
        self.prob = None
        self.h = None
        self.orEnv.SetViewer('qtcoin')

        print "start"
        self.orEnv.SetDebugLevel(DebugLevel.Verbose)
        self.orEnv.Reset()

        self.orEnv.Load("../ormodels/human_wpi.xml")

        print "draw frame"
        T = MakeTransform( eye(3), transpose(matrix([0,0,0]))) 
        self.h = misc.DrawAxes( self.orEnv, matrix(T), 1 )

        print "try to create problem"
        self.prob = RaveCreateProblem(self.orEnv,'Prediction')

    def listen(self):
        print "trying to listen"
        self.prob.SendCommand('StartListening')

if __name__ == "__main__":
    print "main function"
    p = prediction_subscriber()
    p.listen()
    k=0
    while 1:
        k

