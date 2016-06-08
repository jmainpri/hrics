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

import os
from os import path
from shutil import copyfile
import numpy as np
import math

# Import the base class
from generate_gmm_data_single import *


class HumanJointGmmTraining(HumanGmmTraining):
    def __init__(self):
        HumanGmmTraining.__init__(self)
        self.human = ["human_one", "human_two"]
        return

    def load_active_traj_files(self, trajs_dir, traj_files):
        active_dofs_class_trajs = []
        for f in traj_files:
            traj = np.loadtxt(open(trajs_dir + f, "rb"), delimiter=",")
            active_traj = self.get_active_traj(traj)
            active_dofs_class_trajs.append(active_traj)
        return active_dofs_class_trajs

    # Goes over the human trajectories in the gmm data for all
    # classes and generates active trajectories
    # Note: This is only performed for the human_one
    def load_human_active_dofs_trajs(self, human_id):

        classes_trajs = []

        for id, c in enumerate(self.classes):
            class_dir = ("gmm_data/classes/" + self.human[human_id] +
                         "/class_" + str(id) + "/")
            traj_files = os.listdir(class_dir)
            classes_trajs.append(
                self.load_active_traj_files(class_dir, traj_files))

        return classes_trajs

    # Load the test trajs
    def load_human_active_dofs_testing_trajs(self, human_id):
        trajs_dir = "testing/" + self.human[human_id] + "/"
        # sorting will keep the indices correct
        trajs_files = sorted(os.listdir(trajs_dir))
        return [self.load_active_traj_files(trajs_dir, trajs_files)]

    # Save active DoFs trajectories to file
    # The motions are re-sampled and saved as a compound files
    # Note: This is only performed for the human_one
    def save_trajs_to_compound_file(self, humans_class_trajs, filename):

        nb_humans = len(humans_class_trajs)
        if nb_humans < 1:
            print "Error: the number of humans is not at least 1"
            return

        nb_classes = len(humans_class_trajs[0])
        nb_active_dofs = len(self.active_dofs)

        for id_class in range(nb_classes):

            nb_trajs_in_class = len(humans_class_trajs[0][id_class])

            # Creates the joint compound matrix
            #      there is one such matrix for each of the classes
            #
            # rows : the default length of the trajectories
            # cols : nb of active DoFs times the nb of humans
            class_trajs = np.zeros(
                (self.traj_default_length * nb_trajs_in_class,
                 nb_humans * nb_active_dofs))

            for id_human in range(nb_humans):

                id_beg_dof = nb_active_dofs * id_human
                id_end_dof = id_beg_dof + nb_active_dofs

                # Re-sample the trajectories and sets ids instead of time
                for id_traj, traj in enumerate(
                        humans_class_trajs[id_human][id_class]):

                    indices = get_n_samples(traj.shape[0],
                                            self.traj_default_length)

                    # For each configuration in the array
                    for i, idx in enumerate(indices):
                        # Sets the configuration
                        q = traj[idx, :]
                        id_row = id_traj * self.traj_default_length + i
                        class_trajs[id_row, id_beg_dof:id_end_dof] = q

                        # Replaces the time by ids (do not use true time)
                        class_trajs[id_row, id_beg_dof] = i

            class_file_name = filename + str(id_class) + ".csv"
            print "save trajs to : {class_file_name}".format(**locals())
            np.savetxt(class_file_name, class_trajs,
                       delimiter=",", fmt='%10.5f')

    def generate_compound_joint_training(self):
        human_1_active_trajs = self.load_human_active_dofs_trajs(0)
        human_2_active_trajs = self.load_human_active_dofs_trajs(1)

        filename = "gmm_data/classes/compound_joint/compound_"
        self.save_trajs_to_compound_file(
            [human_1_active_trajs, human_2_active_trajs], filename)

    def generate_compound_joint_testing(self):
        human_1_active_trajs = self.load_human_active_dofs_testing_trajs(0)
        human_2_active_trajs = self.load_human_active_dofs_testing_trajs(1)
        # self.save_trajs_to_compound_file(
        #     [human_1_active_trajs, human_2_active_trajs])
        print len(human_1_active_trajs)
        print len(human_2_active_trajs)

        filename = "gmm_data/testing/compound_joint/compound_"
        self.save_trajs_to_compound_file(
            [human_1_active_trajs, human_2_active_trajs], filename)


# run the server
if __name__ == "__main__":
    gmm_data = HumanJointGmmTraining()

    # This is before learning the GMM
    gmm_data.copy_training_files()
    gmm_data.generate_compound_joint_training()
    gmm_data.generate_compound_joint_testing()

    # This is valid after learning the GMM and testing
    gmm_data.generate_gmr_trajectories()
    gmm_data.generate_gmr_predicted_trajectories()