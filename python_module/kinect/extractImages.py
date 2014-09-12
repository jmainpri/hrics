#!/usr/bin/env python
import os, csv
from shutil import copyfile

dir = "/home/rafihayne/workspace/statFiles/recorded_motion/"

files = [
    "[1368 - 1445] motion_saved_00000_00000.csv",
    "[1525 - 1582] motion_saved_00000_00000.csv",
    "[1662 - 1711] motion_saved_00000_00000.csv",
    "[1813 - 1864] motion_saved_00000_00000.csv",
    "[2421 - 2473] motion_saved_00000_00000.csv",
    "[2567 - 2643] motion_saved_00000_00000.csv",
    "[2726 - 2762] motion_saved_00000_00000.csv",
    "[2882 - 2946] motion_saved_00000_00000.csv",
    "[3026 - 3092] motion_saved_00000_00000.csv",
    "[3913 - 3978] motion_saved_00000_00000.csv",
    "[4074 - 4119] motion_saved_00000_00000.csv",
    "[4224 - 4279] motion_saved_00000_00000.csv",
    "[4519 - 4566] motion_saved_00000_00000.csv",
    "[4784 - 4834] motion_saved_00000_00000.csv",
    "[4919 - 4971] motion_saved_00000_00000.csv",
    "[5062 - 5107] motion_saved_00000_00000.csv",
    "[5209 - 5278] motion_saved_00000_00000.csv",
    "[5879 - 5948] motion_saved_00000_00000.csv",
    "[6052 - 6124] motion_saved_00000_00000.csv",
    "[6380 - 6426] motion_saved_00000_00000.csv",
    "[6521 - 6573] motion_saved_00000_00000.csv",
    "[6788 - 6840] motion_saved_00000_00000.csv",
    "[6924 - 6961] motion_saved_00000_00000.csv",
    "[7218 - 7280] motion_saved_00000_00000.csv",
    "[7357 - 7423] motion_saved_00000_00000.csv",
    "[8475 - 8532] motion_saved_00000_00000.csv",
    "[8663 - 8726] motion_saved_00000_00000.csv",
    "[8843 - 8907] motion_saved_00000_00000.csv",
    "[8992 - 9058] motion_saved_00000_00000.csv",
    "[9316 - 9382] motion_saved_00000_00000.csv",
    "[9494 - 9543] motion_saved_00000_00000.csv",
]

def getImages(filein, outpath):

    path,filename = os.path.split(filein)

    with open(filein, 'rb') as csvIn:
        reader = csv.reader(csvIn)
        for row in reader:
            s = row[-3]
            ms = row[-2]

            imagefile = "0_"+s+"_"+ms+".png"

            if os.path.exists(path+'/../images/'+imagefile):
                print "copying: " + imagefile
                copyfile(path+'/../images/'+imagefile, outpath+imagefile)

for file in files:
    count = 0
    #getImages(dir+file)
    print count
    count += 1
                    




