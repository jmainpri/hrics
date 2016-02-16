#!/usr/bin/python

import os
import shutil
import sys
import csv
import list_selected
import numpy as np


# computes the limits of the dofs
def compute_limits_file(filename):
    # Loads the matix from file
    x = np.loadtxt(filename, delimiter=',')
    print "filename : ", filename, " : ", x.shape

    max_col = np.array(x.shape[1] * [0.])
    min_col = np.array(x.shape[1] * [0.])

    for i in range(x.shape[1]):
        max_col[i] = np.amax(x[:, i])
        min_col[i] = np.amin(x[:, i])

    return [max_col, min_col]


def set_max_values(current, new_values):
    for i in range(len(current)):
        if current[i] < new_values[i]:
            current[i] = new_values[i]
    return current


def set_min_values(current, new_values):
    for i in range(len(current)):
        if current[i] > new_values[i]:
            current[i] = new_values[i]
    return current


def find_max_and_min_in_folder(folder, max_values, min_values):
    files = os.listdir(folder)
    for f in files:
        [max_new, min_new] = compute_limits_file(folder + f)
        print "max : ", max_new
        print "min : ", min_new
        if max_values is None and min_values is None:
            max_values = max_new
            min_values = min_new
        else:
            max_values = set_max_values(max_values, max_new)
            min_values = set_min_values(min_values, min_new)
            # print "max_values : ", max_values
            #print "min_values : ", min_values
    return [max_values, min_values]

# This script copies the files to a new folder
# with two folder human_one and human_two
# it can optionaly restore the human ordering
if __name__ == "__main__":

    max_v = None
    min_v = None

    compute_human_experiment = False

    if compute_human_experiment:

        [max_v, min_v] = find_max_and_min_in_folder('training/human_one/',
                                                    max_v, min_v)
        [max_v, min_v] = find_max_and_min_in_folder('training/human_two/',
                                                    max_v, min_v)
        [max_v, min_v] = find_max_and_min_in_folder('testing/human_one/', max_v,
                                                    min_v)
        [max_v, min_v] = find_max_and_min_in_folder('testing/human_two/', max_v,
                                                    min_v)

        print "max_values : ", max_v
        print "min_values : ", max_v

        np.savetxt("max_dof.csv", max_v, delimiter=",")
        np.savetxt("min_dof.csv", min_v, delimiter=",")
    else:

        [max_v, min_v] = find_max_and_min_in_folder(
            'first_motions_icra/human_one/', max_v, min_v)
        [max_v, min_v] = find_max_and_min_in_folder(
            'first_motions_icra/human_two/', max_v, min_v)

        print "max_values : ", max_v
        print "min_values : ", max_v

        np.savetxt("icra_max_dof.csv", max_v, delimiter=",")
        np.savetxt("icra_min_dof.csv", min_v, delimiter=",")