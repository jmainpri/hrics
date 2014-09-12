#
# Copyright (c) 2014 WPI/ARC
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software for any purpose
# with or without   fee is hereby granted, provided   that the above  copyright
# notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS  SOFTWARE INCLUDING ALL  IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS. IN NO EVENT SHALL THE AUTHOR  BE LIABLE FOR ANY SPECIAL, DIRECT,
# INDIRECT, OR CONSEQUENTIAL DAMAGES OR  ANY DAMAGES WHATSOEVER RESULTING  FROM
# LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
# OTHER TORTIOUS ACTION,   ARISING OUT OF OR IN    CONNECTION WITH THE USE   OR
# PERFORMANCE OF THIS SOFTWARE.
#
#                                            Jim Mainprice on Tuesday Aug 5 2014

# Code adapted from KAREN TROY's code

from math_utils import *
from numpy import *
from numpy import linalg as la
from TransformMatrix import *
from rodrigues import *
from copy import deepcopy

class BioHumanIk():

    def __init__(self):

        # Matrices
        self.t_trans = matrix(eye(4))
        self.trunkE = None
        self.UAE = None
        self.LAE = None
        self.handE = None

        # Mode
        self.use_elbow_pads = False

    def get_markers_in_pelvis_frame(self, markers, t_pelvis):

        # Construct frame centered at torso with orientation
        # set by the pelvis frame, add rotation offset for matlab code

        trunk_center = (markers[0] + markers[1])/2

        self.t_trans = deepcopy(t_pelvis)
        self.t_trans[0:3, 3] = transpose(matrix(trunk_center))
        self.t_trans = self.t_trans * MakeTransform(rodrigues([0, 0, pi]), matrix([0, 0, 0]))

        inv_t_trans = la.inv(self.t_trans)

        points_3d = len(markers)*[array([0., 0., 0.])]

        for i, p in enumerate(markers):
            points_3d[i] = array(array(inv_t_trans).dot(array(append(p, 1.0))))[0:3]
            # points_3d[i] *= 1000

        return points_3d

    def compute_ik(self, markers, t_elbow=None):

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

        # Get joint centers and axises

        # TORSO
        trunk_origin = markers[1]
        xyphoid_T8 = markers[0] - markers[1]
        trunk_center = markers[0] - 0.5*xyphoid_T8  # torso_origin
        C7_sternal = markers[2] - markers[3]
        c7s_midpt = markers[3] + 0.5*C7_sternal

        # SHOULDER
        fixedz = markers[4][2]  # - 10  # 10 -> 1 cm
        acromion = array([markers[4][0], markers[4][1], fixedz])
        gleno_center = array([acromion[0], acromion[1], markers[5][2]])  # p_shoulder_center

        if not self.use_elbow_pads:

            # ELBOW
            elb_axis = markers[6] - markers[7]
            elb_center = markers[7] + 0.5 * elb_axis  # p_elbow_center

            # HAND
            wrist_axis = markers[9] - markers[8]
            wrist_center = markers[8] + 0.5 * wrist_axis  # p_wrist_center
            UlnStylPro = markers[8] + 10 * wrist_axis / la.norm(wrist_axis)
            LApY = elb_center - wrist_center  # UlnStylPro
            hand_origin = markers[10]  # - array([0.0, 0.0, 40.0])

        else:

            elb_center = array(transpose(t_elbow[:, 3]).tolist()[0][:3])
            elb_axis = array(transpose(t_elbow[:, 2]).tolist()[0][:3])
            # print "t_elbow : "
            # print t_elbow
            # print "elb_axis : "
            # print elb_axis
            # print "elb_center : "
            # print elb_center

            # HAND
            wrist_axis = markers[7] - markers[6]
            wrist_center = markers[6] + 0.5 * wrist_axis  # p_wrist_center
            UlnStylPro = markers[6] + 10 * wrist_axis / la.norm(wrist_axis)
            LApY = elb_center - wrist_center  # UlnStylPro
            hand_origin = markers[8] - array([0.0, 0.0, 0.04]) # TODO perform offset in wrist frame

        # --------------------------------------------------------------------
        # Define matrices

        # TRUNK
        trunkY = c7s_midpt-trunk_center
        trunkZ = cross(trunkY, xyphoid_T8)
        trunkX = cross(trunkY, trunkZ)
        trunkX *= - 1.0
        trunkZ *= - 1.0
        trunkE = normalize(transpose(matrix([trunkX, trunkY, trunkZ])))

        # for each rotation matrix verify U*Ut = eye(3)
        # print "trunkE"
        # print trunkE * transpose(trunkE)

        # SHOULDER
        UAY = gleno_center - elb_center
        UAZ = cross(UAY, elb_axis)  # / la.norm(elb_axis)  # - UAZ_offset
        UAX = cross(UAY, UAZ)
        UAE = normalize(transpose(matrix([UAX, UAY, UAZ])))

        # ELBOW
        LAY = LApY
        LAX = cross(LAY, wrist_axis)
        LAZ = cross(LAX, LAY)
        LAE = normalize(transpose(matrix([LAX, LAY, LAZ])))
        # LAE = t_elbow[0:3][:, 0:3]

        # HAND
        handY = wrist_center - hand_origin
        handX = cross(handY, wrist_axis)
        handZ = cross(handX, handY)
        handE = normalize(transpose(matrix([handX, handY, handZ])))

        # Store matrices for drawing
        self.trunkE = self.t_trans * MakeTransform(trunkE, matrix(trunk_center))
        self.UAE = self.t_trans * MakeTransform(UAE, matrix(gleno_center))
        self.LAE = self.t_trans * MakeTransform(LAE, matrix(elb_center))
        self.handE = self.t_trans * MakeTransform(handE, matrix(wrist_center))

        # --------------------------------------------------------------------
        # Translations
        # d_torso_shoulder = (gleno_center - trunk_center)
        d_shoulder_elbow = la.norm((gleno_center - elb_center))
        d_elbow_wrist = la.norm((wrist_center - elb_center))

        # Get shoulder center in the torso frame
        # get it the global frame then compute the torso frame
        inv_torso = la.inv(matrix(MakeTransform(trunkE, matrix([0., 0., 0.]))))
        d_torso = array(array(inv_torso).dot(append((gleno_center), 1)))[0:3]

        # --------------------------------------------------------------------
        # Global frame
        globalE = matrix([[-1.0, 0.0, 0.0], [0.0, 0.0, 1], [0.0, 1.0, 0.0]])
        # this is simply a reflection of how our subjects were positioned relative to global
        # globalE=[1 0 0; 0 0 1; 0 -1 0]; # change for points defined in pelvis frame
        # globalE=[0 1 0; 0 0 1; 1 0 0]

        # --------------------------------------------------------------------
        # Get eulers angles from matrices

        # Method 1: find euler angles
        trunk_about_glob = la.inv(globalE) * trunkE
        trunk_about_glob = normalize(trunk_about_glob)
        tr_a = euler_from_matrix(trunk_about_glob, 'rxzy')
        tr_a = tr_a * 180/math.pi

        UA_about_trunk = la.inv(trunkE) * UAE
        UA_about_trunk = normalize(UA_about_trunk)
        sh_a = euler_from_matrix(UA_about_trunk, 'ryxy')
        sh_a = sh_a * 180/math.pi

        LA_about_UA = la.inv(UAE) * LAE
        LA_about_UA = normalize(LA_about_UA)
        elb_a = euler_from_matrix(LA_about_UA, 'rzxy')
        elb_a = elb_a * 180/math.pi

        # calculate euler angles for the wrist
        hand_about_LA = la.inv(LAE) * handE
        hand_about_LA = normalize(hand_about_LA)
        wrist_a = euler_from_matrix(hand_about_LA, 'rzxy')
        wrist_a = wrist_a * 180/math.pi

        # --------------------------------------------------------------------
        # Pack arm configuration in degrees
        q = array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q[0] = 0
        q[1] = tr_a[0]
        q[2] = tr_a[1]
        q[3] = tr_a[2]
        q[4] = sh_a[0]
        q[5] = sh_a[1]
        q[6] = sh_a[2]
        q[7] = elb_a[0]
        q[8] = elb_a[1]
        q[9] = elb_a[2]
        q[10] = wrist_a[0]
        q[11] = wrist_a[1]
        q[12] = wrist_a[2]
        return [q, d_torso, d_shoulder_elbow, d_elbow_wrist]
