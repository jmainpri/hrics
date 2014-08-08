#!/usr/bin/python

from MocapDrawer import *
from MocapCommon import *
from BioHumanIk import *

import sys
import time
from copy import deepcopy

class TestBioHumanIk(BioHumanIk):

    def __init__(self, m_file, o_file):

        BioHumanIk.__init__(self)

        NB_HUMAN    = 2
        ELBOW_PADS  = True
        RARM_ONLY   = True
        NB_MARKERS = get_nb_markers(ELBOW_PADS, RARM_ONLY)

        self.drawer = Drawer(NB_MARKERS, NB_HUMAN, ELBOW_PADS, RARM_ONLY)
        self.drawer.load_file(m_file, o_file)

        self.use_elbow_pads = ELBOW_PADS

        self.env = self.drawer.env
        self.env.Load("../../ormodels/humans_bio_env.xml")
        self.humans = self.env.GetRobots()

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
        human.SetTransform(array(t_pelvis * t_offset))

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
    def get_human_configuration(self, config):

        motion = [config]
        for configuration in motion:  # motion should be one row. otherwise take the last element

            q = self.human.GetDOFValues()
            for i, dof in enumerate(configuration):
                if mapping[i] >= 0:
                    self.q[mapping[i]] = dof * pi / 180
        return q

    def play_skeleton(self):
        # for frame in self.frames:
        prev_time = self.drawer.frames[0].get_time()

        for i, frame in enumerate(self.drawer.frames):

            self.drawer.clear()
            self.drawer.draw_frame_skeleton(frame)

            curr_time = frame.get_time()
            dt = curr_time - prev_time
            prev_time = curr_time
            time.sleep(dt)

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

                markers = self.get_markers_in_pelvis_frame(markers, t_pelvis)
                [q, d_torso, d_shoulder_elbow, d_elbow_wrist] = self.compute_ik(markers, t_elbow)

                offset_pelvis_torso = array([0., 0., 0.])

                self.set_human_model_sizes(h, t_pelvis, offset_pelvis_torso,
                                           d_torso, d_shoulder_elbow, d_elbow_wrist)

            print len(markers)

            if i % 3 == 0:
                print "press enter to continue"
                sys.stdin.readline()

    def run(self):

        self.play_skeleton()

if __name__ == "__main__":

    # folder = '/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/python_module/bioik/data/vacation/test/'
    # m_file = folder + '[0600-1000]markers.csv'
    # o_file = folder + '[0600-1000]objects.csv'

    folder = '/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/python_module/bioik/data/vacation/four_motions/'
    m_file = folder + '[0580-0680]markers.csv'
    o_file = folder + '[0580-0680]objects.csv'

    m_file = folder + '[0760-0900]markers.csv'
    o_file = folder + '[0760-0900]objects.csv'

    m_file = folder + '[1060-1180]markers.csv'
    o_file = folder + '[1060-1180]objects.csv'

    # m_file = folder + '[1300-1420]markers.csv'
    # o_file = folder + '[1300-1420]objects.csv'

    m_file = folder + '[2160-2280]markers.csv'
    o_file = folder + '[2160-2280]objects.csv'

    # m_file = folder + '[1820-1960]markers.csv'
    # o_file = folder + '[1820-1960]objects.csv'

    print "try to load file : ", m_file
    print "try to load file : ", o_file

    test = TestBioHumanIk(m_file, o_file)
    test.run()

    while True:
        print "press enter to exit"
        sys.stdin.readline()