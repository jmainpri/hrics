#!/usr/bin/python

# openrave-robot.py ../../ormodels/human_wpi_bio.xml --info=joints

from TestBioHumanIk import *

import sys
import time
from copy import deepcopy


class TestTwoArmIk(TestBioHumanIk):

    def __init__(self, name):

        TestBioHumanIk.__init__(self, name)

        self.nb_humans      = 1
        self.rarm_only      = False
        self.use_elbow_pads = False
        self.compute_left_arm = True
        self.environment_file = "../../ormodels/humans_env_two_arms.xml"

if __name__ == "__main__":

    data_folder = '/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/python_module/bioik/data/'

    folder = data_folder + 'two_arm_test_data/'
    name = "[0800-3511]"
    m_file = folder + name + 'markers.csv'
    o_file = folder + name + 'objects.csv'
    
    print "try to load file : ", m_file
    print "try to load file : ", o_file

    test = TestTwoArmIk(name)
    test.initialize(m_file, o_file)

    test.mapping.append(test.humans[0].GetJoint("lShoulderY1").GetDOFIndex())
    test.mapping.append(test.humans[0].GetJoint("lShoulderX").GetDOFIndex())
    test.mapping.append(test.humans[0].GetJoint("lShoulderY2").GetDOFIndex())
    test.mapping.append(test.humans[0].GetJoint("lElbowZ").GetDOFIndex())
    test.mapping.append(test.humans[0].GetJoint("lElbowX").GetDOFIndex())
    test.mapping.append(test.humans[0].GetJoint("lElbowY").GetDOFIndex())
    test.mapping.append(test.humans[0].GetJoint("lWristZ").GetDOFIndex())
    test.mapping.append(test.humans[0].GetJoint("lWristX").GetDOFIndex())
    test.mapping.append(test.humans[0].GetJoint("lWristY").GetDOFIndex())

    print len(test.mapping)

    while True:
        test.run()
        test.save_file()
        print "press enter to exit"
        sys.stdin.readline()

