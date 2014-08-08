#!/usr/bin/python

from MocapDrawer import *
from MocapCommon import *
from BioHumanIk import *

import sys
import time
from copy import deepcopy

mapping = [-1, 6, 7, 8, 16, 17, 18, 20, 21, 22, 24, 25, 26]

class TestBioHumanIk(BioHumanIk):

    def __init__(self, name, m_file, o_file):

        BioHumanIk.__init__(self)

        NB_HUMAN    = 2
        ELBOW_PADS  = True
        RARM_ONLY   = True
        NB_MARKERS = get_nb_markers(ELBOW_PADS, RARM_ONLY)

        self.name = name

        self.drawer = Drawer(NB_MARKERS, NB_HUMAN, ELBOW_PADS, RARM_ONLY)
        self.drawer.load_file(m_file, o_file)

        self.use_elbow_pads = ELBOW_PADS

        self.env = self.drawer.env
        self.env.Load("../../ormodels/humans_bio_env.xml")
        self.humans = self.env.GetRobots()
        self.handles = []

        self.traj_human1 = []
        self.traj_human2 = []

        # Get torso offset
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

    def remap(markers, in_markers):

        out_markers = deepcopy(in_markers)

        # Remap wrist
        tmp = out_markers[6]
        out_markers[6] = in_markers[7]
        out_markers[7] = tmp

        return out_markers

    def set_human_model_sizes(self, human, t_pelvis, offset_pelvis_torso,
                              offset_torso, offset_shoulder_elbow, offset_elbow_wrist):
        # Place the human according to the torso and pelvis frame
        # future notes: when placing the human according to the pelvis frame
        # the torso offset should be applied
        t_offset = MakeTransform(eye(3), matrix(offset_pelvis_torso))
        human.SetTransform(array(eye(4)))

        t_waist = t_pelvis * t_offset

        waist_center = array(transpose(t_waist[:, 3]).tolist()[0][:3])
        waist_rot = euler_from_matrix(t_waist, 'rxyz')

        human.SetDOFValues(waist_rot, [3, 4, 5])
        human.SetDOFValues(waist_center, [0, 1, 2])

        # Set elbow size
        # self.human.SetDOFValues([-self.offset_torso[0]], [9])
        # self.human.SetDOFValues([self.offset_torso[1]], [10])
        # self.human.SetDOFValues([-self.offset_torso[2]], [11])
        human.SetDOFValues([offset_torso[0]], [9])
        human.SetDOFValues([offset_torso[1]], [10])
        human.SetDOFValues([offset_torso[2]], [11])
        human.SetDOFValues([offset_shoulder_elbow], [19])
        human.SetDOFValues([offset_elbow_wrist], [23])

        # Map the joint angles and set to radians
    def get_human_configuration(self, human, config):

        motion = [config]
        for configuration in motion:  # motion should be one row. otherwise take the last element

            q = human.GetDOFValues()
            for i, dof in enumerate(configuration):
                if mapping[i] >= 0:
                    q[mapping[i]] = dof * pi / 180
        return q

    def compute_dist_to_points(self, human, markers, t_elbow):

        # Get joint centers
        p_torso_origin = (markers[0] + markers[1])/2
        p_shoulder_center = array([markers[4][0], markers[4][1], markers[5][2]])
        p_elbow_center = array(transpose(t_elbow[:, 3]).tolist()[0][:3])
        p_wrist_center = (markers[6] + markers[7])/2

        # Get the points in the global frame
        # inv_pelvis = la.inv(t_pelvis)
        inv_pelvis = eye(4)

        p0 = array(array(inv_pelvis).dot(append(p_torso_origin, 1)))[0:3]
        p1 = array(array(inv_pelvis).dot(append(p_shoulder_center, 1)))[0:3]
        p2 = array(array(inv_pelvis).dot(append(p_elbow_center, 1)))[0:3]
        p3 = array(array(inv_pelvis).dot(append(p_wrist_center, 1)))[0:3]

        dist = 0.0

        for j in human.GetJoints():

            p_link = j.GetHierarchyChildLink().GetTransform()[0:3, 3]

            if j.GetName() == "TorsoZ":
                # self.handles.append(self.env.plot3(p_link, pointsize=0.02, colors=array([0, 0, 0]), drawstyle=1))
                dist = la.norm(p_link - p0)
                print "p_link : ", p_link
                print "dist torso : ", dist
                self.handles.append(self.env.plot3(p_link, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
                # l = self.human.GetLink("TorsoDummyY")
                # self.handles.append(misc.DrawAxes(self.env, j.GetHierarchyChildLink().GetTransform(), 1.0))
            if j.GetName() == "rShoulderX":
                # self.handles.append(self.env.plot3(p_link, pointsize=0.05, colors=array([0, 0, 0]), drawstyle=1))
                dist = la.norm(p_link - p1)
                print "dist shoulder : ", dist
                self.handles.append(self.env.plot3(p_link, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
            if j.GetName() == "rElbowZ":
                dist = la.norm(p_link - p2)
                print "dist elbow : ", dist
                self.handles.append(self.env.plot3(p_link, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
            if j.GetName() == "rWristX":
                dist = la.norm(p_link - p3)
                self.handles.append(self.env.plot3(p_link, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
                print "dist wrist : ", dist
            #if j.GetName() == "rShoulderZ":
            #    self.handles.append(misc.DrawAxes(self.env, j.GetHierarchyChildLink().GetTransform(), 0.3))

        # for j in self.human.GetJoints():
        #     if j.GetName() == "TorsoX":  # j.GetName() == "rShoulderX" or
        #         t_link = j.GetHierarchyChildLink().GetTransform()
        #         self.handles.append(misc.DrawAxes(self.env, t_link, 0.3))

        # self.handles.append(self.env.plot3(p1, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
        # self.handles.append(self.env.plot3(p2, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
        # self.handles.append(self.env.plot3(p3, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))

        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("TorsoZ").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("zTorsoTrans").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rShoulderY").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rElbowZ").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rWristY").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rArmTrans").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, inv_pelvis * self.t_torso, 1))
        # self.handles.append(misc.DrawAxes(self.env, self.t_pelvis, 2))
        # self.handles.append(misc.DrawAxes(self.env, eye(4), 2))
        # self.handles.append(misc.DrawAxes(self.env, self.t_pelvis, 2))

        # print "joint : ", self.human.GetJoint("zTorsoTrans").GetHierarchyChildLink().GetTransform()[0:3, 3]

        # self.handles.append(misc.DrawAxes(self.env, self.trunkE, .2))
        # self.handles.append(misc.DrawAxes(self.env, self.UAE, .2))
        # self.handles.append(misc.DrawAxes(self.env, self.LAE, .2))
        # self.handles.append(misc.DrawAxes(self.env, self.handE, .2))

        return dist

    def save_file(self):

        print "Trying to output new file"

        #  Get the out filename
        # dir, path = os.path.split(self.m_filepath)
        # name, type = path.rsplit('.', 1)
        # m_outpath = name + '_fixed.' + type
        #
        # dir, path = os.path.split(self.o_filepath)
        # name, type = path.rsplit('.', 1)
        # o_outpath = name + '_fixed.' + type

        # No need to normalize ids.  we output marker names
        # print "Trying to normalize ids"
        # self.normalize_ids()

        traj_file_human1 = self.name + '_human1_.csv'
        traj_file_human2 = self.name + '_human2_.csv'

        with open(traj_file_human1, 'w') as h1_file:
            with open(traj_file_human2, 'w') as h2_file:

                line_str = ''
                for q in self.traj_human1:
                    for q_i in q:
                        for q_ii in q_i:
                            line_str += str(q_ii) + ','
                    line_str = line_str.rstrip(',')
                    line_str += '\n'
                h1_file.write(line_str)

                line_str = ''
                for q in self.traj_human2:
                    for q_i in q:
                        for q_ii in q_i:
                            line_str += str(q_ii) + ','
                    line_str = line_str.rstrip(',')
                    line_str += '\n'
                h2_file.write(line_str)
        print "End writing !!!"

    def play_skeleton(self):
        # for frame in self.frames:
        prev_time = self.drawer.frames[0].get_time()

        self.traj_human1 = []
        self.traj_human2 = []

        for i, frame in enumerate(self.drawer.frames):

            curr_time = frame.get_time()
            dt = curr_time - prev_time
            prev_time = curr_time
            time.sleep(dt)

            del self.handles[:]
            self.drawer.clear()
            self.drawer.draw_frame_skeleton(frame)

            humans = self.drawer.isolate_humans(frame)

            for j, h in enumerate(self.humans):

                markers_remaped = self.remap(humans[j].markers)

                markers = []
                for m in markers_remaped:
                    markers.append(m.array)

                transforms = []
                for o in humans[j].objects:
                    transforms.append(o.get_transform())

                t_pelvis = transforms[0] * MakeTransform(rodrigues([0, 0, pi]), matrix([0, 0, 0]))
                t_head   = transforms[1]
                t_elbow  = transforms[2]
                t_elbow  = t_elbow * MakeTransform(rodrigues([0, 0, -pi/2]), matrix([0, 0, 0]))
                t_elbow  = t_elbow * MakeTransform(rodrigues([0, pi/2, 0]), matrix([0, 0, 0]))

                # self.handles.append(misc.DrawAxes(self.env, t_elbow, .2))

                trunk_center = (markers[0] + markers[1])/2
                inv_pelvis = la.inv(t_pelvis)
                trunk_center = array(array(inv_pelvis).dot(append(trunk_center, 1)))[0:3]

                markers_in_pelvis = self.get_markers_in_pelvis_frame(markers, t_pelvis)

                [config, d_torso, d_shoulder_elbow, d_elbow_wrist] = self.compute_ik(markers_in_pelvis, la.inv(self.t_trans) * t_elbow)

                offset_pelvis_torso = trunk_center
                offset_pelvis_torso -= self.offset_pelvis_torso_init
                # offset_pelvis_torso += array([0.16, 0., 0.])

                self.set_human_model_sizes(h, t_pelvis, offset_pelvis_torso,
                                           d_torso, d_shoulder_elbow, d_elbow_wrist)

                q_cur = self.get_human_configuration(h, config)
                h.SetDOFValues(q_cur[0:h.GetDOF()])

                self.compute_dist_to_points(h, markers, t_elbow)

                # Save to current configurations
                if j == 0:
                    traj = self.traj_human1
                if j == 1:
                    traj = self.traj_human2

                traj.append([[dt], h.GetDOFValues()])

                print curr_time

            # if i % 3 == 0:
            #     print "press enter to continue"
            #     sys.stdin.readline()

    def run(self):

        self.play_skeleton()

if __name__ == "__main__":

    # folder = '/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/python_module/bioik/data/vacation/test/'
    # m_file = folder + '[0600-1000]markers.csv'
    # o_file = folder + '[0600-1000]objects.csv'

    folder = '/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/python_module/bioik/data/vacation/four_motions/'

    # m_file = folder + '[0580-0680]markers.csv'
    # o_file = folder + '[0580-0680]objects.csv'
    #
    # m_file = folder + '[0760-0900]markers.csv'
    # o_file = folder + '[0760-0900]objects.csv'
    #
    # m_file = folder + '[1060-1180]markers.csv'
    # o_file = folder + '[1060-1180]objects.csv'

    # m_file = folder + '[1300-1420]markers.csv'
    # o_file = folder + '[1300-1420]objects.csv'

    # m_file = folder + '[2160-2280]markers.csv'
    # o_file = folder + '[2160-2280]objects.csv'

    # m_file = folder + '[1820-1960]markers.csv'
    # o_file = folder + '[1820-1960]objects.csv'

    # m_file = folder + '[1500-1680]markers.csv'
    # o_file = folder + '[1500-1680]objects.csv'

    # LATEST ------------------------------------
    # m_file = folder + '[0440-0580]markers.csv'
    # o_file = folder + '[0440-0580]objects.csv'

    # m_file = folder + '[0700-0860]markers.csv'
    # o_file = folder + '[0700-0860]objects.csv'

    m_file = folder + '[1460-1620]markers.csv'
    o_file = folder + '[1460-1620]objects.csv'
    name = '[1460-1620]'

    # m_file = folder + '[1800-1980]markers.csv'
    # o_file = folder + '[1800-1980]objects.csv'
    #
    # m_file = folder + '[1840-2300]markers.csv'
    # o_file = folder + '[1840-2300]objects.csv'

    print "try to load file : ", m_file
    print "try to load file : ", o_file

    test = TestBioHumanIk(name, m_file, o_file)

    while True:
        test.run()
        test.save_file()
        print "press enter to exit"
        sys.stdin.readline()