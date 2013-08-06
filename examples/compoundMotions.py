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

    motionClass1 = ["/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_0.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_1.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_2.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_3.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_4.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_5.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_6.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_7.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_8.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_9.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_10.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_11.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_12.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_13.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_14.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_15.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_16.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_17.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_18.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_19.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_20.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_21.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_22.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_23.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/0/resample/resampled_24.csv",
]

    motionClass2 = ["/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_0.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_1.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_2.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_3.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_4.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_5.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_6.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_7.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_8.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_9.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_10.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_11.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_12.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_13.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_14.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_15.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_16.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_17.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_18.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_19.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_20.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_21.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_22.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_23.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/1/resample/resampled_24.csv",]

    motionClass3 = ["/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_0.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_1.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_2.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_3.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_4.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_5.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_6.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_7.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_8.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_9.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_10.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_11.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_12.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_13.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_14.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_15.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_16.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_17.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_18.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_19.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_20.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_21.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_22.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_23.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/2/resample/resampled_24.csv",]

    motionClass4 = ["/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_0.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_1.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_2.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_3.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_4.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_5.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_6.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_7.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_8.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_9.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_10.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_11.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_12.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_13.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_14.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_15.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_16.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_17.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_18.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_19.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_20.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_21.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_22.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_23.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/3/resample/resampled_24.csv",
]

    motionClass5 = ["/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_0.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_1.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_2.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_3.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_4.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_5.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_6.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_7.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_8.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_9.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_10.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_11.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_12.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_13.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_14.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_15.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_16.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_17.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_18.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_19.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_20.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_21.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_22.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_23.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/4/resample/resampled_24.csv",]

    motionClass6 = ["/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_0.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_1.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_2.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_3.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_4.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_5.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_6.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_7.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_8.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_9.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_10.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_11.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_12.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_13.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_14.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_15.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_16.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_17.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_18.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_19.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_20.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_21.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_22.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_23.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/5/resample/resampled_24.csv",
]

    motionClass7 = ["/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_0.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_1.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_2.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_3.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_4.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_5.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_6.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_7.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_8.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_9.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_10.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_11.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_12.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_13.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_14.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_15.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_16.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_17.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_18.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_19.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_20.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_21.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_22.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_23.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/6/resample/resampled_24.csv",
]

    motionClass8 = ["/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_0.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_1.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_2.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_3.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_4.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_5.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_6.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_7.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_8.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_9.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_10.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_11.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_12.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_13.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_14.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_15.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_16.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_17.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_18.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_19.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_20.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_21.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_22.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_23.csv",
"/home/rafihayne/workspace/statFiles/recorded_motion/Library/7/resample/resampled_24.csv",]


    motionClasses = [motionClass1, motionClass2, motionClass3, motionClass4, 
                    motionClass5, motionClass6, motionClass7, motionClass8]

    compound(motionClasses)









