#!/usr/bin/env python
import os, csv

#!/usr/bin/env python
import os, csv

def filterFields(fullRow):
    newRow = []
    newRow.append( fullRow[0] ) #PelvisTransX
    newRow.append( fullRow[1] ) #PelvisTransY
    newRow.append( fullRow[2] ) #PelvisTransZ
    newRow.append( fullRow[3] ) #PelvisRotX
    newRow.append( fullRow[6] ) #TorsoX
    newRow.append( fullRow[7] ) #TorsoY
    newRow.append( fullRow[8] ) #TorsoZ
    newRow.append( fullRow[12] ) #rShoulderX
    newRow.append( fullRow[13] ) #rShoulderY
    newRow.append( fullRow[14] ) #rShoulderZ
    newRow.append( fullRow[15] ) #rArmTrans
    newRow.append( fullRow[16] ) #rElbowZ

    return newRow


def compound(motionClasses):
    dir = os.path.dirname(motionClasses[0][0])

    for i in range(len(motionClasses)):
        compoundFile = dir+'/compound_'+str(i)+'.csv'

        with open(compoundFile, 'wb') as csvoutput:
            for motion in motionClasses[i]:
                with open(motion, 'rb') as csvinput:    
                    writer = csv.writer(csvoutput)

                    for j, row in enumerate(csv.reader(csvinput)): #j is the index needed by matlab
                        if row:                                    #remove blank row at the end of files.
                            row = filterFields(row)
                            row.insert(0, j)
                            writer.writerow(row)

    
if __name__ == "__main__":
    print "main function"

    motionClass1 = ['/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00000.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00001.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00002.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00003.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00004.csv']

    motionClass2 = ['/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00005.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00006.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00007.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00008.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00009.csv']

    motionClass3 = ['/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00010.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00011.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00012.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00013.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00014.csv']

    motionClass4 = ['/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00015.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00016.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00017.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00018.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00019.csv']

    motionClass5 = ['/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00020.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00021.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00022.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00023.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00024.csv']

    motionClass6 = ['/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00025.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00026.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00027.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00028.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00029.csv']

    motionClass7 = ['/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00030.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00031.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00032.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00033.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00034.csv']

    motionClass8 = ['/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00035.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00036.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00037.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00038.csv',
                    '/home/rafihayne/workspace/statFiles/recorded_motion/test/motion_saved_00000_00039.csv']


    motionClasses = [motionClass1, motionClass2, motionClass3, motionClass4, 
                    motionClass5, motionClass6, motionClass7, motionClass8]

    compound(motionClasses)









