#!/usr/bin/env python
import os
import shutil


#for root, dirs, filenames in os.walk("/home/rafihayne/workspace/statFiles/recorded_motion/Library"):
#    for f in filenames:
        #log = open(os.path.join(root, f),'r')
#        print f


dir = "/home/rafihayne/workspace/statFiles/recorded_motion/"
filenames = []       


for i in range(0, 200):
    filenames.append(dir + "motion_saved_00000_"+str(i).zfill(5)+".csv")

for i, file in enumerate(filenames):
    outfile = dir + "Library/" + str(i%8) 
    shutil.move(file, outfile)
   
