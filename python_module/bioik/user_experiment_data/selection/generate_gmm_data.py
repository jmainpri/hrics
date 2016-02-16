#!/usr/bin/env python

# Copyright (c) 2015 Max Planck Institute
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
#                                    Jim Mainprice on Wednesday January 03 2016

# ACTIVE DOFS FOR BIO MODEL
# ID(0) : Joint(1), Dof : 6, Pelvis(min = 1.92, max = 2.08) : 0.17
# ID(1) : Joint(1), Dof : 7, Pelvis(min = 0.84, max = 1.03) : 0.18
# ID(2) : Joint(1), Dof : 8, Pelvis(min = 1.09, max = 1.30) : 0.22
# ID(3) : Joint(1), Dof : 9, Pelvis(min = -0.05, max = 0.05) : 0.10
# ID(4) : Joint(1), Dof : 10, Pelvis(min = -0.05, max = 0.05) : 0.10
# ID(5) : Joint(1), Dof : 11, Pelvis(min = 1.82, max = 2.57) : 0.75
# ID(6) : Joint(2), Dof : 12, TorsoX(min = -0.10, max = 0.26) : 0.36
# ID(7) : Joint(3), Dof : 13, TorsoZ(min = -0.51, max = 0.04) : 0.54
# ID(8) : Joint(4), Dof : 14, TorsoY(min = -0.04, max = 0.55) : 0.60
# ID(9) : Joint(8), Dof : 18, rShoulderTransX(min = 0.05, max = 0.16) : 0.11
# ID(10) : Joint(9), Dof : 19, rShoulderTransY(min = 0.12, max = 0.25) : 0.13
# ID(11) : Joint(10), Dof : 20, rShoulderTransZ(min = 0.06, max = 0.19) : 0.13
# ID(12) : Joint(11), Dof : 21, rShoulderY1(min = -1.09, max = 0.97) : 2.06
# ID(13) : Joint(12), Dof : 22, rShoulderX(min = -1.48, max = -0.26) : 1.22
# ID(14) : Joint(13), Dof : 23, rShoulderY2(min = -1.14, max = 1.39): 2.53
# ID(15) : Joint(14), Dof : 24, rArmTrans(min = 0.27, max = 0.45): 0.18
# ID(16) : Joint(15), Dof : 25, rElbowZ(min = 0.51, max = 2.04): 1.53
# ID(17) : Joint(16), Dof : 26, rElbowX(min = -0.47, max = 0.32): 0.79
# ID(18) : Joint(17), Dof : 27, rElbowY(min = -3.16, max = 3.16): 6.32
# ID(19) : Joint(18), Dof : 28, rForeArmTrans(min = 0.20, max = 0.30): 0.10
# ID(20) : Joint(19), Dof : 29, rWristZ(min = -0.96, max = 0.60): 1.56
# ID(21) : Joint(20), Dof : 30, rWristX(min = -0.34, max = 0.96): 1.30
# ID(22) : Joint(21), Dof : 31, rWristY(min = -0.25, max = 0.37): 0.62

# MAP from
import os
from os import path
from shutil import copyfile
import numpy as np
import math


# Returns [nb_sample] evenly placed integers in the interval [0 length]
def get_n_samples(length, nb_sample):
    inc = float(length) / float(nb_sample)
    k = 0.
    indices = []
    for j in range(nb_sample):
        indices.append(int(math.floor(k)))
        k += inc
    return indices


class HumanGmmTraining:
    def __init__(self):

        self.herakles_bio_openrave_map = {}
        self.herakles_bio_openrave_map["Time"] = -1
        self.herakles_bio_openrave_map["PelvisTransX"] = 0
        self.herakles_bio_openrave_map["PelvisTransY"] = 1
        self.herakles_bio_openrave_map["PelvisTransZ"] = 2
        self.herakles_bio_openrave_map["PelvisRotX"] = 3
        self.herakles_bio_openrave_map["PelvisRotY"] = 4
        self.herakles_bio_openrave_map["PelvisRotZ"] = 5
        self.herakles_bio_openrave_map["TorsoX"] = 6
        self.herakles_bio_openrave_map["TorsoZ"] = 7
        self.herakles_bio_openrave_map["TorsoY"] = 8
        self.herakles_bio_openrave_map["rShoulderTransX"] = 9
        self.herakles_bio_openrave_map["rShoulderTransY"] = 10
        self.herakles_bio_openrave_map["rShoulderTransZ"] = 11
        self.herakles_bio_openrave_map["rShoulderY1"] = 16
        self.herakles_bio_openrave_map["rShoulderX"] = 17
        self.herakles_bio_openrave_map["rShoulderY2"] = 18
        self.herakles_bio_openrave_map["rArmTrans"] = 19
        self.herakles_bio_openrave_map["rElbowZ"] = 20
        self.herakles_bio_openrave_map["rElbowX"] = 21
        self.herakles_bio_openrave_map["rElbowY"] = 22
        self.herakles_bio_openrave_map["rForeArmTrans"] = 23
        self.herakles_bio_openrave_map["rWristZ"] = 24
        self.herakles_bio_openrave_map["rWristX"] = 25
        self.herakles_bio_openrave_map["rWristY"] = 26

        # Set the translation dofs
        self.translation_dofs = ["PelvisTransX", "PelvisTransY", "PelvisTransZ"]

        # Add an offset for the time
        for key in self.herakles_bio_openrave_map:
            self.herakles_bio_openrave_map[key] += 1

        for key in self.herakles_bio_openrave_map:
            print key + " : " + str(self.herakles_bio_openrave_map[key])

        self.active_dofs = []
        self.active_dofs.append("Time")
        self.active_dofs.append("PelvisTransX")
        self.active_dofs.append("PelvisTransY")
        self.active_dofs.append("PelvisTransZ")
        # self.active_dofs.append("PelvisRotX")
        # self.active_dofs.append("PelvisRotY")
        self.active_dofs.append("PelvisRotZ")
        self.active_dofs.append("TorsoX")
        self.active_dofs.append("TorsoZ")
        self.active_dofs.append("TorsoY")
        self.active_dofs.append("rShoulderY1")
        self.active_dofs.append("rShoulderX")
        self.active_dofs.append("rShoulderY2")
        self.active_dofs.append("rElbowZ")
        self.active_dofs.append("rElbowX")
        self.active_dofs.append("rElbowY")
        self.active_dofs.append("rWristZ")
        self.active_dofs.append("rWristX")
        self.active_dofs.append("rWristY")

        for j, dof in enumerate(self.active_dofs):
            print "j= {j} and dof = {dof}".format(**locals())

        self.classes = []

        # 0     2     2     2     2     2     2     2     2     2 (1)
        # 1     2     2     2     2     2     2     2     2     2 (1)
        # 2     2     2     2     2     2     2     2     2     2 (1)
        # 3     4     4     4     4     4     4     4     4     4 (1)
        # 4     2     2     2     2     2     2     2     2     2 (1)
        # 5     2     2     2     2     2     2     2     2     2 (1)
        # 6     2     2     2     2     2     2     2     2     2 (1)
        # 7     4     4     4     4     4     4     4     4     4 (1)
        # 8     2     2     2     2     2     2     2     2     2 (0)
        # 9     4     4     4     4     4     4     4     4     4 (1)
        # 10    4     4     4     4     4     4     4     4     4 (1)
        # 11    4     4     4     4     4     4     4     4     4 (1)
        # 12    3     3     3     3     3     3     3     3     3 (1)
        # 13    2     2     2     2     2     2     2     2     2 (0)
        # 14    2     2     2     2     2     2     2     2     2 (0)
        # 15    4     4     4     4     4     4     4     4     4 (1)
        # 16    4     4     4     4     4     4     4     4     4 (1)
        # 17    2     2     2     2     2     2     2     2     2 (0)
        # 18    4     4     4     4     4     4     4     4     4 (1)
        # 19    4     4     4     4     4     4     4     4     4 (1)
        # 20    4     4     4     4     4     4     4     4     4 (1)

        # Selection classification
        # Class : 2 : 0 from file [0612-0703]_human1_.csv
        # Class : 2 : 1 from file [0904-1027]_human1_.csv
        # Class : 2 : 2 from file [0921-1010]_human1_.csv
        # Class : 4 : 3 from file [1018-1131]_human1_.csv
        # Class : 2 : 4 from file [1159-1255]_human1_.csv
        # Class : 2 : 5 from file [1197-1363]_human1_.csv
        # Class : 2 : 6 from file [1248-1428]_human1_.csv
        # Class : 4 : 7 from file [1496-1591]_human1_.csv
        # Class : 4 : 8 from file [1595-1694]_human1_.csv
        # Class : 4 : 9 from file [1639-1779]_human1_.csv
        # Class : 4 : 10 from file [1648-1802]_human1_.csv
        # Class : 4 : 11 from file [1809-1897]_human1_.csv
        # Class : 3 : 12 from file [1881-1970]_human1_.csv
        # Class : 3 : 13 from file [1896-2006]_human1_.csv
        # Class : 3 : 14 from file [2020-2116]_human1_.csv
        # Class : 4 : 15 from file [2142-2234]_human1_.csv
        # Class : 4 : 16 from file [2259-2349]_human1_.csv
        # Class : 3 : 17 from file [2483-2568]_human1_.csv
        # Class : 4 : 18 from file [2550-2642]_human1_.csv
        # Class : 4 : 19 from file [2556-2697]_human1_.csv
        # Class : 4 : 20 from file [3124-3230]_human1_.csv

        # CLASS 1
        class_bottom_grey_to_green = [""] * 9
        class_bottom_grey_to_green[0] = "0235-0413]"
        class_bottom_grey_to_green[1] = "[0256-0408]"
        class_bottom_grey_to_green[2] = "[0282-0414]"
        class_bottom_grey_to_green[3] = "[0310-0416]"
        class_bottom_grey_to_green[4] = "[0335-0444]"
        class_bottom_grey_to_green[5] = "[0336-0518]"
        class_bottom_grey_to_green[6] = "[0398-0540]"
        class_bottom_grey_to_green[7] = "[0433-0586]"
        class_bottom_grey_to_green[8] = "[0512-0674]"

        # CLASS 2
        class_grey_to_yellow = []
        class_grey_to_yellow.append("[0426-0511]")
        class_grey_to_yellow.append("[0451-0556]")
        class_grey_to_yellow.append("[0542-0804]")
        class_grey_to_yellow.append("[0579-0693]")
        class_grey_to_yellow.append("[0584-0685]")
        class_grey_to_yellow.append("[0593-0731]")
        class_grey_to_yellow.append("[0604-0737]")
        class_grey_to_yellow.append("[1984-2084]")
        class_grey_to_yellow.append("[0637-0847]")
        class_grey_to_yellow.append("[0697-0845]")
        class_grey_to_yellow.append("[0743-0850]")
        class_grey_to_yellow.append("[0743-0863]")
        class_grey_to_yellow.append("[0810-0932]")
        class_grey_to_yellow.append("[0821-1046]")
        class_grey_to_yellow.append("[1051-1151]")
        class_grey_to_yellow.append("[1066-1182]")
        class_grey_to_yellow.append("[1354-1503]")
        class_grey_to_yellow.append("[1399-1487]")
        class_grey_to_yellow.append("[1497-1680]")
        class_grey_to_yellow.append("[1525-1626]")
        class_grey_to_yellow.append("[1567-1674]")
        class_grey_to_yellow.append("[1735-1906]")
        class_grey_to_yellow.append("[2007-2096]")
        class_grey_to_yellow.append("[2040-2133]")
        class_grey_to_yellow.append("[2122-2244]")
        class_grey_to_yellow.append("[2273-2397]")
        class_grey_to_yellow.append("[2286-2423]")
        class_grey_to_yellow.append("[2410-2498]")
        class_grey_to_yellow.append("[2454-2586]")
        class_grey_to_yellow.append("[2531-2748]")
        class_grey_to_yellow.append("[2643-2792]")
        class_grey_to_yellow.append("[2664-2909]")
        class_grey_to_yellow.append("[2780-2951]")

        # CLASS 3
        class_grey_to_red = []
        class_grey_to_red.append("[1583-1672]")
        class_grey_to_red.append("[1719-1831]")
        class_grey_to_red.append("[1851-2018]")
        class_grey_to_red.append("[1992-2088]")
        class_grey_to_red.append("[2122-2232]")

        # CLASS 4
        class_top_grey_to_green = []
        class_top_grey_to_green.append("[0660-0767]")
        class_top_grey_to_green.append("[0676-0760]")
        class_top_grey_to_green.append("[0825-0929]")
        class_top_grey_to_green.append("[0830-0908]")
        class_top_grey_to_green.append("[0847-0947]")
        class_top_grey_to_green.append("[1094-1188]")
        class_top_grey_to_green.append("[1118-1248]")
        class_top_grey_to_green.append("[1190-1297]")
        class_top_grey_to_green.append("[1272-1383]")
        class_top_grey_to_green.append("[1305-1576]")
        class_top_grey_to_green.append("[1342-1446]")
        class_top_grey_to_green.append("[1349-1459]")
        class_top_grey_to_green.append("[1375-1457]")
        class_top_grey_to_green.append("[1460-1559]")
        class_top_grey_to_green.append("[1549-1672]")
        class_top_grey_to_green.append("[1656-1826]")
        class_top_grey_to_green.append("[1861-1937]")
        class_top_grey_to_green.append("[1877-1949]")
        class_top_grey_to_green.append("[1877-1977]")
        class_top_grey_to_green.append("[2117-2269]")
        class_top_grey_to_green.append("[2168-2352]")
        class_top_grey_to_green.append("[2258-2343]")
        class_top_grey_to_green.append("[2296-2397]")
        class_top_grey_to_green.append("[2539-2636]")
        class_top_grey_to_green.append("[3074-3171]")
        class_top_grey_to_green.append("[3576-3681]")

        self.classes.append(class_bottom_grey_to_green)
        self.classes.append(class_grey_to_yellow)
        self.classes.append(class_grey_to_red)
        self.classes.append(class_top_grey_to_green)

        self.traj_default_length = 100

        self.classes_trajs = []
        return

    def get_files(self):

        traj1_dir = "training/human_one/"
        traj2_dir = "training/human_two/"

        for i, c in enumerate(self.classes):
            class1_dir = "gmm_data/classes/human_one/class_" + str(i) + "/"
            class2_dir = "gmm_data/classes/human_two/class_" + str(i) + "/"
            for split in c:
                for traj_dir, class_dir in zip([traj1_dir, traj2_dir],
                                               [class1_dir, class2_dir]):
                    human_files = os.listdir(traj_dir)
                    traj_file = [s for s in human_files if split in s]
                    if len(traj_file) == 1:
                        # Make trajectory temporary folder
                        if not os.path.exists(class_dir):
                            os.makedirs(class_dir)
                        copyfile(traj_dir + traj_file[0],
                                 class_dir + traj_file[0])

    def get_active_traj(self, traj):

        active_traj = np.zeros((traj.shape[0], len(self.active_dofs)))
        for i, q in enumerate(traj):
            if i == 0:
                q_0 = q
            for j, dof in enumerate(self.active_dofs):
                dof_id = self.herakles_bio_openrave_map[dof]
                active_traj[i, j] = q[dof_id]
                if any(dof in s for s in self.translation_dofs):
                    active_traj[i, j] -= q_0[dof_id]

        return active_traj

    def generate_active_dofs_trajs_testing(self):

        traj_dir = "testing/human_one/"
        traj_files = os.listdir(traj_dir)

        testing_active_dir = "gmm_data/testing/"

        if not os.path.exists(testing_active_dir):
            os.makedirs(testing_active_dir)

        test_trajs = np.zeros((self.traj_default_length * len(traj_files),
                               len(self.active_dofs)))

        for id_traj, f in enumerate(sorted(traj_files)):

            print "add testing : {id_traj} from file {f}".format(**locals())

            traj = np.loadtxt(open(traj_dir + f, "rb"), delimiter=",")
            active_traj = self.get_active_traj(traj)

            # sub sample trajectory
            indices = get_n_samples(traj.shape[0], self.traj_default_length)
            for i, idx in enumerate(indices):
                test_trajs[
                id_traj * self.traj_default_length + i, :] = active_traj[idx, :]

            # Set time to ids instead
            for i in range(self.traj_default_length):
                test_trajs[id_traj * self.traj_default_length + i, 0] = i

        save_file_name = testing_active_dir + "compound_test.csv"
        print "save trajs to : {save_file_name}".format(**locals())
        np.savetxt(save_file_name, test_trajs, delimiter=",", fmt='%10.5f')

    def generate_active_dofs_trajs(self):

        self.classes_trajs = []

        for id, c in enumerate(self.classes):

            class_trajs = []
            class_dir = "gmm_data/classes/human_one/class_" + str(id) + "/"
            traj_files = os.listdir(class_dir)

            for f in traj_files:
                traj = np.loadtxt(open(class_dir + f, "rb"), delimiter=",")
                active_traj = self.get_active_traj(traj)
                class_trajs.append(active_traj)

            self.classes_trajs.append(class_trajs)

    def save_class_trajs_to_file(self):

        for id_class, c in enumerate(self.classes_trajs):

            class_dir = "gmm_data/human_one/class_" + str(id_class) + "/"
            class_file_name = (
                "gmm_data/classes/compound_" + str(id_class) + ".csv")

            class_trajs = np.zeros(
                (self.traj_default_length * len(c),
                 len(self.active_dofs)))

            for id_traj, traj in enumerate(c):

                indices = get_n_samples(traj.shape[0], self.traj_default_length)
                for i, idx in enumerate(indices):
                    class_trajs[
                    id_traj * self.traj_default_length + i, :] = traj[idx, :]

                # Set time to ids instead
                for i, idx in enumerate(indices):
                    class_trajs[id_traj * self.traj_default_length + i, 0] = i

            print "save trajs to : {class_file_name}".format(**locals())
            np.savetxt(class_file_name, class_trajs,
                       delimiter=",", fmt='%10.5f')

    def generate_gmr_trajecories(self):

        for id, c in enumerate(self.classes):

            class_human_one_dir = ("gmm_data/classes/human_one/class_"
                                   + str(id) + "/")
            class_human_two_dir = ("gmm_data/classes/human_two/class_" +
                                   str(id) + "/")

            class_gmr_human_one_dir = "gmm_data/gmr_trajs/human_one/"
            class_gmr_human_two_dir = "gmm_data/gmr_trajs/human_two/"
            if not os.path.exists(class_gmr_human_one_dir):
                os.makedirs(class_gmr_human_one_dir)
            if not os.path.exists(class_gmr_human_two_dir):
                os.makedirs(class_gmr_human_two_dir)

            class_gmr_file = ("gmm_data/gmr_trajs/traj_class_"
                              + str(id + 1) + ".csv")
            active_traj = np.loadtxt(open(class_gmr_file, "rb"),
                                     delimiter=",")

            class_gmr_all_dofs_name = (
                "traj_all_dofs_class_" + str(id + 1) + ".csv")

            traj_files = os.listdir(class_human_two_dir)
            copyfile(class_human_two_dir + traj_files[0],
                     class_gmr_human_two_dir + class_gmr_all_dofs_name)

            traj_files = os.listdir(class_human_one_dir)
            traj = np.loadtxt(open(class_human_one_dir + traj_files[0], "rb"),
                              delimiter=",")
            q_0 = np.copy(traj[0, :])
            traj = np.resize(traj, (active_traj.shape[0], traj.shape[1]))

            for i in range(active_traj.shape[0]):

                for j, dof in enumerate(self.active_dofs):
                    if dof == "Time":
                        traj[i, 0] = 0.01
                        continue
                    dof_id = self.herakles_bio_openrave_map[dof]
                    traj[i, dof_id] = active_traj[i, j]
                    if any(dof in s for s in self.translation_dofs):
                        traj[i, dof_id] += q_0[dof_id]

            print "save gmr trajs to : {}".format(
                class_gmr_human_one_dir + class_gmr_all_dofs_name)
            np.savetxt(class_gmr_human_one_dir + class_gmr_all_dofs_name, traj,
                       delimiter=",", fmt='%3.5f')
        return

    def generate_gmr_predicted_trajecories(self):

        classification = np.loadtxt(
            open('gmm_data/gmr_trajs/testing_classification_50.csv', "rb"),
            delimiter=",")

        gmr_trajs = []
        for i in range(len(self.classes)):
            gmr_trajs.append(
                np.loadtxt(open('gmm_data/gmr_trajs/traj_class_' +
                                str(i + 1) + '.csv', "rb"),
                           delimiter=","))

        gmr_predicted_human_one_dir = "gmm_data/gmr_trajs/predicted/"
        if not os.path.exists(gmr_predicted_human_one_dir):
            os.makedirs(gmr_predicted_human_one_dir)

        testing_active_dir = "testing/human_one/"

        # sorting will keep the indicies correct
        traj_files = sorted(os.listdir(testing_active_dir))

        for traj_id, f in enumerate(traj_files):

            move3d_trajectory = np.loadtxt(
                open(testing_active_dir + f, "rb"), delimiter=",")

            # Get the first configuration
            q_0 = np.copy(move3d_trajectory[0, :])

            # Get the classification id
            class_id = int(classification[traj_id] - 1)
            # print "classification[{}] : {}".format(traj_id, class_id)

            predicted_trajectory = gmr_trajs[class_id]
            sampling_mapping = get_n_samples(
                predicted_trajectory.shape[0], move3d_trajectory.shape[0])

            # i   : is the id in the move3d trajectory
            # idx : is the id in the sub-sampled gmm trajectory
            for i, idx in enumerate(sampling_mapping):

                for j, dof in enumerate(self.active_dofs):
                    if dof == "Time":  # Keep timing of the original trajectory
                        continue
                    dof_id = self.herakles_bio_openrave_map[dof]
                    move3d_trajectory[i, dof_id] = predicted_trajectory[idx, j]
                    if any(dof in s for s in self.translation_dofs):
                        move3d_trajectory[i, dof_id] += q_0[dof_id]

            predicted_traj_file_dir = gmr_predicted_human_one_dir + f[:11] + "/"
            if not os.path.exists(predicted_traj_file_dir):
                os.makedirs(predicted_traj_file_dir)

            predicted_traj_file_name = predicted_traj_file_dir + f

            print "save predicted trajs to : {}".format(
                predicted_traj_file_name)

            print "shape : " + str(move3d_trajectory.shape)
            np.savetxt(predicted_traj_file_name, move3d_trajectory,
                       delimiter=",",
                       fmt='%3.5f')
        return


# run the server
if __name__ == "__main__":
    gmm_data = HumanGmmTraining()
    gmm_data.get_files()
    gmm_data.generate_active_dofs_trajs()
    gmm_data.save_class_trajs_to_file()
    gmm_data.generate_gmr_trajecories()
    gmm_data.generate_active_dofs_trajs_testing()
    gmm_data.generate_gmr_predicted_trajecories()
