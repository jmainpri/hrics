#!/usr/bin/python

import os
import shutil
import sys
import csv


def get_splits_from_file(filename):
    # filename = type + ".txt"

    splits = []

    with open(filename, 'rb') as file:
        csv_reader = csv.reader(file, delimiter=',')
        for row in csv_reader:
            splits.append(row[0])

    return splits


if __name__ == "__main__":

    type = ""

    for index in range(1, len(sys.argv)):
        if sys.argv[index] == "-t" and index + 1 < len(sys.argv):
            type = str(sys.argv[index + 1])

    if type == "":
        print "specify the type (-t type)"

    else:
        splits = get_splits_from_file(type + ".txt")
        for i, s in enumerate(splits):
            print "id : " + str(i) + " , " + s
