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
import time
import socket
from copy import deepcopy
from numpy import *
from numpy import linalg as la
from TransformMatrix import *
from rodrigues import *
from subprocess import call
import marker_utils
from math import *

mapping = [-1, 6, 7, 8, 18, 17, 16, 20, 22, 23, 24]

TCP_IP = '127.0.0.1'
TCP_PORT = 5005

# Joint and link names
# -------------------------------------------------------------------------
# TorsoX        6           6         PelvisBody        TorsoDummyX
# TorsoY        7           7         TorsoDummyX       TorsoDummyY
# TorsoZ        8           8         TorsoDummyY       TorsoDummyTransX
# xTorsoTrans   9           9         TorsoDummyTransX  TorsoDummyZ
# yTorsoTrans   10          10        TorsoDummyTransX  TorsoDummyTransY
# zTorsoTrans   11          11        TorsoDummyTransY  TorsoDummyTransZ
# rShoulderX    16          16        TorsoDummyTransZ  rShoulderDummyX
# rShoulderZ    17          17        rShoulderDummyX   rShoulderDummyZ
# rShoulderY    18          18        rShoulderDummyZ   rHumerus
# rArmTrans     19          19        rHumerus          rElbowDummy1
# rElbowZ       20          20        rElbowDummy1      rRadius
# rForearmTrans 21          21        rRadius           rWristDummy
# rWristX       22          22        rWristDummy       rWristDummyX
# rWristY       23          23        rWristDummyX      rWristDummyY
# rWristZ       24          24        rWristDummyY      rHand

# Reminder of marker order
# -------------------------------------------------------------------------
# 0 xyphoid process
# 1 T8
# 2 sternal notch
# 3 C7
# 4 Acromion process
# 5 Glenohumeral cntr of rot. (post)
# 6 Medial epicondyle
# 7 lateral epicondyle
# 8 ulnar styloid
# 9 radial styloid
# 10 2nd metacarpal head


class BioHumanIk():

    def __init__(self):

        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.Reset()
        self.env.Load("../ormodels/human_wpi_bio.xml")

        self.human = self.env.GetRobots()[0]

        # Get pelvis torso offset and set human to position
        self.offset_pelvis_torso_init = self.human.GetJoint("TorsoX").GetHierarchyChildLink().GetTransform()[0:3, 3]

        self.handles = []

        # Should store the marker set
        self.markers = None
        self.q = None

        self.offset_pelvis_torso = None
        self.offset_torso_shoulder = None
        self.offset_shoulder_elbow = None
        self.offset_elbow_wrist = None
        self.t_torso = None
        self.t_pelvis = None

    # Marker array should be the size of the number of markers
    # the markers should be ordered as described in the reminder
    def set_markers(self, markers):

        self.markers = markers

    def get_markers_in_frame(self, t_0):

        inv_torso = la.inv(t_0)

        points_3d = len(self.markers)*[array([0, 0, 0])]

        for i, p in enumerate(self.markers):
            points_3d[i] = array(array(inv_torso).dot(array(append(p, 1.0))))[0:3]

        return points_3d

    def get_markers_in_pelvis_frame(self):

        # Construct frame centered at torso with orientation
        # set by the pelvis frame, add rotation offset for matlab code
        t_trans = deepcopy(self.t_pelvis)
        t_trans[0:3, 3] = deepcopy(self.t_torso[0:3, 3])
        t_trans = t_trans * MakeTransform(rodrigues([0, 0, pi]), matrix([0, 0, 0]))
        t_trans = la.inv(t_trans)

        points_3d = len(self.markers)*[array([0, 0, 0])]

        for i, p in enumerate(self.markers):
            points_3d[i] = array(array(t_trans).dot(array(append(p, 1.0))))[0:3]
            points_3d[i] *= 1000

        return points_3d

    def save_markers_to_file(self):

        with open("./matlab/markers_tmp.csv", 'w') as m_file:

            line_str = ""

            # Construct frame centered at torso with orientation
            # set by the pelvis frame, add rotation offset for matlab code
            t_trans = deepcopy(self.t_pelvis)
            t_trans[0:3, 3] = deepcopy(self.t_torso[0:3, 3])
            t_trans = t_trans * MakeTransform(rodrigues([0, 0, pi]), matrix([0, 0, 0]))

            # inv_pelvis = la.inv(self.t_pelvis)
            # self.handles.append(misc.DrawAxes(self.env, inv_pelvis * t_trans, 2))

            t_trans = la.inv(t_trans)

            for marker in self.markers:

                marker = array(array(t_trans).dot(append(marker, 1)))[0:3]

                line_str += str(marker[0]*1000) + ','
                line_str += str(marker[1]*1000) + ','
                line_str += str(marker[2]*1000) + ','

            line_str = line_str.rstrip(',')
            line_str += '\n'
            m_file.write(line_str)

    def set_pelvis_frame(self, t_pelvis):

        # Get pelvis frame in world

        # print "t_pelvis : ", t_pelvis
        # print "t_pelvis[3:7] ", t_pelvis[3:7]
        # print "t_pelvis[0:3] ", t_pelvis[0:3]

        mat = MakeTransform(rotationMatrixFromQuat(array(t_pelvis[3:7])), matrix(t_pelvis[0:3]))

        new_x = -array(transpose(mat[:, 2]).tolist()[0][:3])  # get z vector from matrix
        new_x[2] = 0
        new_x /= la.norm(new_x)

        new_z = array([0.0, 0.0, 1.0])

        new_y = cross(new_x, new_z)
        new_y /= la.norm(new_y)
        new_y = -new_y

        mat[:, 0][:3] = transpose(array([new_x]))
        mat[:, 1][:3] = transpose(array([new_y]))
        mat[:, 2][:3] = transpose(array([new_z]))

        self.t_pelvis = matrix(mat)

        # Compute the offset between the trunk and the pelvis
        # the offset_pelvis_torso_init is hard coded and the human model

        trunk_center = (self.markers[0] + self.markers[1])/2

        inv_pelvis = la.inv(self.t_pelvis)
        trunk_center = array(array(inv_pelvis).dot(append(trunk_center, 1)))[0:3]

        self.offset_pelvis_torso = trunk_center
        self.offset_pelvis_torso -= self.offset_pelvis_torso_init

        # print "offset_pelvis_torso : ", self.offset_pelvis_torso

    def set_model_size(self):

        torso_origin = (self.markers[0] + self.markers[1])/2
        p_shoulder_center = array([self.markers[4][0], self.markers[4][1], self.markers[5][2]])
        p_elbow_center = (self.markers[6] + self.markers[7])/2
        p_wrist_center = (self.markers[9] - self.markers[8])/2 + self.markers[8]

        self.offset_torso_shoulder = p_shoulder_center - torso_origin
        self.offset_shoulder_elbow = la.norm(p_shoulder_center - p_elbow_center)
        self.offset_elbow_wrist = la.norm(p_wrist_center - p_elbow_center)

        # Get shoulder center in the torso frame
        # get it the global frame
        # then compute the torso frame

        xyphoid_t8 = self.markers[0] - self.markers[1]
        trunk_center = self.markers[0] - 0.5*xyphoid_t8
        c7_sternal = self.markers[2] - self.markers[3]
        c7s_midpt = self.markers[3] + 0.5*c7_sternal

        mat = MakeTransform(eye(3), matrix(trunk_center))

        new_y = c7s_midpt - trunk_center
        new_z = cross(new_y, xyphoid_t8)
        new_x = cross(new_y, new_z)

        mat[:, 0][:3] = transpose(array([new_x / la.norm(new_x)]))
        mat[:, 1][:3] = transpose(array([new_y / la.norm(new_y)]))
        mat[:, 2][:3] = transpose(array([new_z / la.norm(new_z)]))

        # self.handles.append(misc.DrawAxes(self.env, inv_pelvis * mat, 1))

        # Get shoulder center in the torso frame
        self.t_torso = matrix(mat)
        inv_torso = la.inv(self.t_torso)
        offset_torso = array(array(inv_torso).dot(append(p_shoulder_center, 1)))[0:3]

        # Place the human according to the torso and pelvis frame
        # future notes: when placing the human according to the pelvis frame
        # the torso offset should be applied
        t_offset = MakeTransform(eye(3), matrix(self.offset_pelvis_torso))
        self.human.SetTransform(array(self.t_pelvis * t_offset))

        # Set elbow size
        self.human.SetDOFValues([-offset_torso[0]], [9])
        self.human.SetDOFValues([offset_torso[1]], [10])
        self.human.SetDOFValues([-offset_torso[2]], [11])
        self.human.SetDOFValues([self.offset_shoulder_elbow], [19])
        self.human.SetDOFValues([self.offset_elbow_wrist], [21])

    def compute_dist_to_points(self, frame_id=0):

        # Get joint centers
        p_torso_origin = (self.markers[0] + self.markers[1])/2
        p_shoulder_center = array([self.markers[4][0], self.markers[4][1], self.markers[5][2]])
        p_elbow_center = (self.markers[6] + self.markers[7])/2
        p_wrist_center = (self.markers[9] - self.markers[8])/2 + self.markers[8]

        # Get the points in the global frame
        # inv_pelvis = la.inv(self.t_pelvis)
        inv_pelvis = eye(4)

        p0 = array(array(inv_pelvis).dot(append(p_torso_origin, 1)))[0:3]
        p1 = array(array(inv_pelvis).dot(append(p_shoulder_center, 1)))[0:3]
        p2 = array(array(inv_pelvis).dot(append(p_elbow_center, 1)))[0:3]
        p3 = array(array(inv_pelvis).dot(append(p_wrist_center, 1)))[0:3]

        dist = 0.0

        for j in self.human.GetJoints():

            p_link = j.GetHierarchyChildLink().GetTransform()[0:3, 3]

            if j.GetName() == "TorsoZ":
                # self.handles.append(self.env.plot3(p_link, pointsize=0.02, colors=array([0, 0, 0]), drawstyle=1))
                dist = la.norm(p_link - p0)
                print "p_link : ", p_link
                print "dist torso : ", dist
                # l = self.human.GetLink("TorsoDummyY")
                # self.handles.append(misc.DrawAxes(self.env, j.GetHierarchyChildLink().GetTransform(), 1.0))
            if j.GetName() == "rShoulderX":
                # self.handles.append(self.env.plot3(p_link, pointsize=0.05, colors=array([0, 0, 0]), drawstyle=1))
                dist = la.norm(p_link - p1)
                print "dist shoulder : ", dist
            if j.GetName() == "rElbowZ":
                self.handles.append(self.env.plot3(p_link, pointsize=0.02, colors=array([0, 0, 0]), drawstyle=1))
                dist = la.norm(p_link - p2)
                print "dist elbow : ", dist
            if j.GetName() == "rWristX":
                self.handles.append(self.env.plot3(p_link, pointsize=0.02, colors=array([0, 0, 0]), drawstyle=1))
                dist = la.norm(p_link - p3)
                print "dist wrist : ", dist
            #if j.GetName() == "rShoulderZ":
            #    self.handles.append(misc.DrawAxes(self.env, j.GetHierarchyChildLink().GetTransform(), 0.3))

        # for j in self.human.GetJoints():
        #     if j.GetName() == "TorsoX":  # j.GetName() == "rShoulderX" or
        #         t_link = j.GetHierarchyChildLink().GetTransform()
        #         self.handles.append(misc.DrawAxes(self.env, t_link, 0.3))

        self.handles.append(self.env.plot3(p1, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
        self.handles.append(self.env.plot3(p2, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
        self.handles.append(self.env.plot3(p3, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))

        self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("TorsoZ").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("zTorsoTrans").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rShoulderY").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rElbowZ").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rWristY").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, inv_pelvis * self.t_torso, 1))
        # self.handles.append(misc.DrawAxes(self.env, self.t_pelvis, 2))
        # self.handles.append(misc.DrawAxes(self.env, eye(4), 2))
        self.handles.append(misc.DrawAxes(self.env, self.t_pelvis, 2))

        # print "joint : ", self.human.GetJoint("zTorsoTrans").GetHierarchyChildLink().GetTransform()[0:3, 3]

        return dist

    def draw_markers(self, config=None):

        del self.handles[:]

        # Get the points in the global frame
        # inv_pelvis = la.inv(self.t_pelvis)
        t_0 = eye(4)

        points = self.get_markers_in_frame(t_0)

        colors = []
        nb_points = len(points)
        for n in linspace(0.0, 1.0, num=nb_points):
            colors.append((float(n)*1, (1-float(n))*1, 0))

        points_3d = squeeze(points)

        self.handles.append(self.env.plot3(points=points_3d, pointsize=0.02, colors=array(colors), drawstyle=1))

        q_cur = self.get_human_configuration(config)
        self.human.SetDOFValues(q_cur[0:self.human.GetDOF()])
        # print "(plane of elevation, ", q_cur[16]*180/pi, " , elevation, ", q_cur[17]*180/pi, \
        #     " , axial rotation, ", q_cur[18]*180/pi, ")"

        # draws center of joints points
        self.set_model_size()
        # sys.stdin.readline()

        self.compute_dist_to_points()

    # Map the joint angles and set to radians
    def get_human_configuration(self, config=None):

        if config is None:
            motion = genfromtxt('./matlab/outputik.csv', delimiter=',')
            motion = delete(motion, 0, axis=0)  # Remove first row...
        else:
            motion = [config]

        print "---------------------------"
        # print "motion : ", motion

        for configuration in motion:  # motion should be one row. otherwise take the last element

            self.q = self.human.GetDOFValues()

            for i, dof in enumerate(configuration):
                if mapping[i] >= 0:
                    self.q[mapping[i]] = dof * pi / 180
                if mapping[i] == 16:
                    self.q[mapping[i]] = -self.q[mapping[i]]
                if mapping[i] == 17:
                    self.q[mapping[i]] = -self.q[mapping[i]]
                if mapping[i] == 18:
                    self.q[mapping[i]] = -self.q[mapping[i]]
                if mapping[i] == 22:
                    self.q[mapping[i]] = -self.q[mapping[i]]
                if mapping[i] == 23:
                    self.q[mapping[i]] = -self.q[mapping[i]]
                if mapping[i] == 24:
                    self.q[mapping[i]] = -self.q[mapping[i]]
        return self.q

    def rtocarda( self, R, i, j, k):

        #RTOCARDA (Spacelib): Rotation  matrix  to Cardan or Eulerian angles.
        #
        # Extracts the Cardan (or Euler) angles from a rotation matrix.
        # The  parameters  i, j, k  specify   the   sequence   of  the rotation axes
        # (their value must be the constant (X,Y or Z).
        # j must be different from i and k, k could be equal to i.
        # The two solutions are stored in the  three-element vectors q1 and q2.
        # RTOCARDA performs the inverse operation than CARDATOR.
        # Usage:
        #
        #			[q1,q2]=rtocarda(R,i,j,k)
        #
        # Related functions : MTOCARDA
        #
        # (c) G.Legnani, C. Moiola 1998; adapted from: G.Legnani and R.Adamini 1993
        #___________________________________________________________________________

        #spheader
        #disp('got this far')
        # if ( i<X | i>Z | j<X | j>Z | k<X | k>Z | i==j | j==k )
        # 	error('Error in RTOCARDA: Illegal rotation axis ')
        # end

        a = array([0.0, 0.0, 0.0])
        b = array([0.0, 0.0, 0.0])

        # print "R : ", R

        if (j-i+3) % 3 == 1:
            sig = 1  # ciclic
        else:
            sig = -1  # anti ciclic

        if i != k:  # Cardanic Convention

            i -= 1
            j -= 1
            k -= 1

            a[0] = atan2(-sig*R[j, k], R[k, k])
            a[1] = asin(sig*R[i, k])
            a[2] = atan2(-sig*R[i, j], R[i, i])

            b[0] = atan2(sig*R[j, k], -R[k, k])
            b[1] = ((pi-asin(sig*R[i, k]) + pi) % 2*pi)-pi
            b[2] = atan2(sig*R[i, j], -R[i, i])

        else:  # Euleriana Convention

            l = 6-i-j

            i -= 1
            j -= 1
            k -= 1
            l -= 1

            a[0] = atan2(R[j, i], -sig*R[l, i])
            a[1] = acos(R[i, i])
            a[2] = atan2(R[i, j], sig*R[i, l])

            b[0] = atan2(-R[j, i], sig*R[l, i])
            b[1] = -acos(R[i, i])
            b[2] = atan2(-R[i, j], -sig*R[i, l])

        # report in degrees instead of radians
        a = a * 180/pi
        b = b * 180/pi

        # print "a : ", a
        # print "b : ", b

        return [a, b]

    def normalize(self, x):

        y = eye(3)

        y[0, :] = x[0, :] / la.norm(x[0, :])
        y[1, :] = x[1, :] / la.norm(x[1, :])
        y[2, :] = x[2, :] / la.norm(x[2, :])

        # important to export as matrix
        return matrix(y)

    def compute_ik(self):

        markers = self.get_markers_in_pelvis_frame()

        # 0 -> 3-5 xyphoid process
        # 1 -> 6-8 T8
        # 2 -> 9-11 sternal notch
        # 3 -> 12-14 C7
        # 4 -> 15-17 Acromion process
        # 5 -> 18-20 Glenohumeral cntr of rot. (post)
        # 6 -> 21-23 Medial epicondyle
        # 7 -> 24-26 lateral epicondyle
        # 8 -> 27-29 ulnar styloid
        # 9 -> 30-32 radial styloid
        # 10 -> 33-35 2nd metacarpal head

        # Get joint centers

        # TORSO
        trunk_origin = markers[1]
        xyphoid_T8 = markers[0] - markers[1]
        trunk_center = markers[0] - 0.5*xyphoid_T8
        C7_sternal = markers[2] - markers[3]
        c7s_midpt = markers[3] + 0.5*C7_sternal

        # SHOULDER
        fixedz = markers[4][2]  # - 10  # 10 -> 1 cm
        acromion = [markers[4][0], markers[4][1], fixedz]
        gleno_center = [acromion[0], acromion[1], markers[5][2]]  # Shoulder center

        # ELBOW
        elb_axis = markers[6] - markers[7]
        elb_center = markers[7] + 0.5 * elb_axis

        # HAND
        wrist_axis = markers[9] - markers[8]
        wrist_center = markers[8] + 0.5 * wrist_axis
        UlnStylPro = markers[8] + 10 * wrist_axis / la.norm(wrist_axis)
        LApY = elb_center - wrist_center  # UlnStylPro
        hand_origin = markers[10]  # - array([0.0, 0.0, 40.0])

        # --------------------------------------------------------------------
        # Define matrices
        trunkY = c7s_midpt-trunk_center
        trunkZ = cross(trunkY, xyphoid_T8)
        trunkX = cross(trunkY, trunkZ)
        trunkX *= - 1.0
        trunkZ *= - 1.0
        trunkE = self.normalize(matrix([trunkX, trunkY, trunkZ]))

        # shoulderX = acromion-c7s_midpt
        # shoulderZ = cross(shoulderX, trunkY)
        # shoulderY = cross(shoulderZ, shoulderX)
        # shouldE = self.normalize(matrix([shoulderX, shoulderY, shoulderZ]))

        # UAZ_offset = array([-0.1601, -0.1286, 0.0411])
        UAZ = - elb_axis / la.norm(elb_axis)  # - UAZ_offset
        UAY = gleno_center - elb_center
        UAX = cross(UAY, UAZ)
        UAE = self.normalize(matrix([UAX, UAY, UAZ]))

        LAY = LApY
        LAX = cross(LAY, wrist_axis)
        LAZ = cross(LAX, LAY)
        LAE = self.normalize(matrix([LAX, LAY, LAZ]))

        handY = wrist_center - hand_origin
        handX = cross(handY, wrist_axis)
        handZ = cross(handX, handY)
        handE = self.normalize(matrix([handX, handY, handZ]))

        # --------------------------------------------------------------------
        # Global frame
        globalE = matrix([[-1.0, 0.0, 0.0], [0.0, 0.0, 1], [0.0, 1.0, 0.0]])
        # this is simply a reflection of how our subjects were positioned relative to global
        # globalE=[1 0 0; 0 0 1; 0 -1 0]; # change for points defined in pelvis frame
        # globalE=[0 1 0; 0 0 1; 1 0 0]

        # --------------------------------------------------------------------
        # Get eulers angles

        # Method 1: find euler angles
        trunk_about_glob = trunkE * la.inv(globalE)
        trunk_about_glob = self.normalize(trunk_about_glob)
        [tr_a, tr_b] = self.rtocarda(trunk_about_glob, 1, 3, 2)

        # calculate euler angles for the shoulder
        # normalize to ensure each has a length of one.
        # Method 1: euler angles (ISB recommendation)
        UA_about_trunk = UAE * la.inv(trunkE)
        UA_about_trunk = self.normalize(UA_about_trunk)
        [sh_a, sh_b] = self.rtocarda(UA_about_trunk, 2, 1, 2)

        LA_about_UA = LAE * la.inv(UAE)
        LA_about_UA = self.normalize(LA_about_UA)
        [elb_a, elb_b] = self.rtocarda(LA_about_UA, 3, 1, 2)
        LAY /= la.norm(LAY)
        UAY /= la.norm(UAY)
        elbowdot = LAY.dot(UAY)
        elb_a[0] = acos(elbowdot)*180/pi

        # calculate euler angles for the wrist
        hand_about_LA = handE * la.inv(LAE)
        hand_about_LA = self.normalize(hand_about_LA)
        [wrist_a, wrist_b] = self.rtocarda(hand_about_LA, 3, 1, 2)

        # wrist has problems with euler angle discontinuities.
        if wrist_a[0] <= -90:
            wrist_a[0] = wrist_a[0]+180
            wrist_a[2] = wrist_a[2]+180

        if wrist_a[0] >= 180:
            wrist_a[0] = wrist_a[0]-180
            wrist_a[2] = wrist_a[2]-180

        # wrist_a(1:2) = wrist_a(1:2)  #default wrist offset is 18.5

        q = array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q[0] = 0
        q[1] = tr_a[0]
        q[2] = tr_a[1]
        q[3] = tr_a[2]
        q[4] = sh_a[0]
        q[5] = sh_a[1]
        q[6] = sh_a[2]
        q[7] = elb_a[0]
        q[8] = wrist_a[0]
        q[9] = wrist_a[1]
        q[10] = wrist_a[2]
        return q

if __name__ == "__main__":

    # Load markers from file
    # raw_markers = genfromtxt('data/points.csv', delimiter=',')
    # raw_markers = delete(raw_markers, 0, axis=0)  # Remove first row
    # raw_markers = delete(raw_markers, s_[0:2], 1)  # Remove two columns
    # raw_markers /= 1000  # Set markers in meters
    # (m,) = raw_markers[0].shape  # number of values in the marker set

    marker_file = '/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/bioik/data/second/markers_fixed_cut.csv'
    object_file = '/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/bioik/data/second/objects_fixed_cut.csv'
    [frames_m, frames_o] = marker_utils.load_file(marker_file, object_file)

    h = BioHumanIk()

    for i in range(len(frames_m)):

        h.set_markers(frames_m[i][0:11])
        h.set_pelvis_frame(frames_o[i][0][0:7])
        h.set_model_size()

        config = h.compute_ik()
        h.draw_markers(config)

        time.sleep(0.01)

        # print "Press return to next."
        # sys.stdin.readline()

    print "press enter to exit"
    sys.stdin.readline()