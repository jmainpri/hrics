#!/usr/bin/python

import os
import shutil
import sys
import csv
import list_selected

# This script copies the files to a new folder
# with two folder human_one and human_two
# it can optionaly restore the human ordering
if __name__ == "__main__":

    nb=5
    flip_humans_id = False
    flip_threshold = 1.6
    only_selected = False
    type = "training"

    for index in range(1, len(sys.argv)):

        if sys.argv[index] == "-type" and index+1 < len(sys.argv):
            type = str(sys.argv[index+1])
        if sys.argv[index] == "-block" and index+1 < len(sys.argv):
            nb = int(sys.argv[index+1])
        if sys.argv[index] == "-flip" and index+1 < len(sys.argv):
            print "flip"
            flip_humans_id = True
            flip_threshold = float(sys.argv[index+1])
        if sys.argv[index] == "-onlyselected":
            only_selected = True

    runs = os.listdir("../block"+str(nb))

    if only_selected:
        selected_splits = list_selected.get_splits_from_file("removed_from_training.txt")
        removed_splits = list_selected.get_splits_from_file("removed_from_testing.txt")
        in_collision_splits = list_selected.get_splits_from_file("demo_in_collision.txt")

        print in_collision_splits

    splits = []
    for r in runs:
        folder = "../block" + str(nb) + "/" + str(r) + "/"
        files = os.listdir(folder)

        for f in files:

            s=f[0:11]

            if only_selected and s in in_collision_splits :
                continue
            if only_selected and not( s in selected_splits ):
                continue
            if only_selected and s in removed_splits:
                continue

            if not s in splits:
                splits.append(s)

                file_human_1 = folder+s+"_human1_.csv"
                file_human_2 = folder+s+"_human2_.csv"

                print folder

                swap = False

                if flip_humans_id:

                    # get the first line of the csv
                    # file and swaps indicies if needed
                    with open(file_human_1, 'rb') as f1:
                        with open(file_human_2, 'rb') as f2:

                            csv_human_1 = csv.reader(f1, delimiter=',')
                            csv_human_2 = csv.reader(f2, delimiter=',')

                            line_human1 = csv_human_1.next()
                            line_human2 = csv_human_2.next()

                            if float(line_human1[1]) < flip_threshold:
                                swap = True

                            if swap:
                                line_human1, line_human2 = line_human2, line_human1

                            print "human 1 (x,y,z) : " \
                                    + line_human1[1] + " , " \
                                    + line_human1[2] + " , " \
                                    + line_human1[3]
                            print "human 2 (x,y,z) : " \
                                     + line_human2[1] + " , " \
                                     + line_human2[2] + " , " \
                                     + line_human2[3]
                    if swap:
                        file_human_1, file_human_2 = file_human_2, file_human_1

                shutil.copy(file_human_1,type+"/human_one")
                shutil.copy(file_human_2,type+"/human_two")

                if swap:
                    os.rename(type+"/human_one/"+s+"_human2_.csv", type+"/human_one/"+s+"_human1_.csv")
                    os.rename(type+"/human_two/"+s+"_human1_.csv", type+"/human_two/"+s+"_human2_.csv")

        # for s in splits:
        #     print s