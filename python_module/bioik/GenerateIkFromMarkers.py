#!/usr/bin/python

# openrave-robot.py ../../ormodels/human_wpi_bio.xml --info=joints

from TestBioHumanIk import *

import sys
import time
from copy import deepcopy


class TestTwoArmIk(TestBioHumanIk):
    def __init__(self, name):
        TestBioHumanIk.__init__(self, name)


if __name__ == "__main__":

    environment_file = "../../ormodels/humans_bio_env.xml"
    nb_humans = 1
    rarm_only = True
    elbow_pads = True
    wrist_pads = True
    data_folder = None
    rarm_only = True

    for index in range(1, len(sys.argv)):
        if sys.argv[index] == "-d" and index + 1 < len(sys.argv):
            data_folder = str(sys.argv[index + 1]) + '/'
        if sys.argv[index] == "-n" and index + 1 < len(sys.argv):
            nb_humans = int(sys.argv[index + 1])
        if sys.argv[index] == "-noelbowpad":
            elbow_pads = False
        if sys.argv[index] == "-nowristpad":
            wrist_pads = False
        if sys.argv[index] == "-botharms":
            rarm_only = False
            environment_file = "../../ormodels/humans_env_two_arms.xml"

    if data_folder is None:

        print "Usage : "
        print " -d /path/to/directory   : sets the directory where the marker and object files are"
        print " -n number_of_humans     : sets the number of humans in the scene"
        print " -noelbowpad             : when no elbow pads are used"
        print " -nowristpad             : when no wrist pads are used"
        print " -botharms               : when both arms are used"

    else:

        m_file = data_folder + 'markers_fixed.csv'
        o_file = data_folder + 'objects_fixed.csv'

        print "try to load file : ", m_file
        print "try to load file : ", o_file

        test = TestTwoArmIk("")
        test.nb_humans = nb_humans
        test.elbow_pads = elbow_pads
        test.wrist_pads = wrist_pads
        test.rarm_only = rarm_only
        test.environment_file = environment_file

        test.initialize(m_file, o_file)

        print len(test.mapping)

        while True:
            test.play_skeleton()
            test.save_file("no_split", data_folder)
            print "press enter to exit"
            sys.stdin.readline()
