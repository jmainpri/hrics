#!/usr/bin/env python
import os, csv

def segment(splits, filein):
    path,filename = os.path.split(filein)

    with open(filein, 'rb') as csvIn:
        for split in splits:
            if (split[0] < split[1]):
                if '1' in filename.split('_')[2]:
                    fileout = path+ '/replan/human_two/' + '['+ str( split[0] ).zfill(4) + '-' + str( split[1] ).zfill(4) + ']' + filename
                else:
                    fileout = path+ '/replan/human_one/' + '['+ str( split[0] ).zfill(4) + '-' + str( split[1] ).zfill(4) + ']' + filename
                with open(fileout, 'wb') as csvOut:
                    writer = csv.writer(csvOut)
                    reader = csv.reader(csvIn)
                    for row in list(reader)[split[0]:split[1]]:
                            writer.writerow(row)
            else:
                print str(split[0]) + " isn't < " + str(split[1]) + " no split possible."

    
if __name__ == "__main__":
    print "main function"
    splits = [(1,17)] 
    filein = '/home/rafihayne/workspace/statFiles/recorded_motion/motion_saved_00000_00000.csv'
    segment(splits, filein)
