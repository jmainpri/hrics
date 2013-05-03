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

#print "HOME : " + os.environ['HOME']
#print "PYTHONPATH : " + os.environ['PYTHONPATH']
#print "OPENRAVE_PLUGINS : " + os.environ['OPENRAVE_PLUGINS']

orEnv = Environment()
h = 0

def launch_kinect():
    
    global orEnv
    global h
    
    orEnv.SetViewer('qtcoin')
    
    print "start"
    orEnv.SetDebugLevel(DebugLevel.Verbose)
    orEnv.Reset()
    #orEnv.Load("../ormodels/human_wpi.xml")
    
    #array([[ 0.04479408, -0.01490569,  0.99888503, -2.46015573],
    #       [-0.98325896, -0.17743756,  0.04144557,  0.13977778],
    #       [ 0.17662195, -0.98401917, -0.0226043 ,  0.57281262],
    #       [ 0.        ,  0.        ,  0.        ,  1.        ]]) 
    
    #0.262839 -0.733602 -0.623389 0.0642694 2.99336 -0.755646 2.81558
    
    #T = MakeTransform(yaw_pitch_roll_rotation([0,0,0]),transpose(matrix([1,0,0])))
    #orEnv.GetViewer().SetCamera([0.262839 -0.733602 -0.623389 0.0642694 2.99336 -0.755646 2.81558])
    #sys.stdin.readline()
    
    print "draw frame"
    T = MakeTransform( eye(3), transpose(matrix([0,0,0]))) 
    h = misc.DrawAxes( orEnv, matrix(T), 1 )
    
    print "try to create problem"
    prob = RaveCreateProblem(orEnv,'Kinect')
    cmdout = prob.SendCommand('load data/lab1.env.xml')
    
    #if cmdout is None:
    #    raveLogWarn('command failed!')
    #else:
    #    cmdout = prob.SendCommand('numbodies')
    #    print 'number of bodies are: ',cmdout
    
    sys.stdin.readline()
    
if __name__ == "__main__":
    print "main function"
    launch_kinect()
    
