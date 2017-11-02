#!/usr/bin/env python
import os, csv


def filterFields(fullRow):
    newRow = []
    newRow.append(fullRow[0])  # PelvisTransX
    newRow.append(fullRow[1])  # PelvisTransY
    newRow.append(fullRow[2])  # PelvisTransZ
    newRow.append(fullRow[5])  # PelvisRotZ
    newRow.append(fullRow[6])  # TorsoX
    newRow.append(fullRow[7])  # TorsoY
    newRow.append(fullRow[8])  # TorsoZ
    newRow.append(fullRow[12])  # rShoulderX
    newRow.append(fullRow[13])  # rShoulderY
    newRow.append(fullRow[14])  # rShoulderZ
    newRow.append(fullRow[15])  # rArmTrans
    newRow.append(fullRow[16])  # rElbowZ

    return newRow


def compound():
    dir = "/home/rafi/Desktop/oct_lib/resampled/"

    for i in range(8):
        compoundFile = dir + 'compound_' + str(i) + '.csv'
        with open(compoundFile, 'wb') as csvoutput:
            for j in range(25):
                motion = dir + "resampled_" + str((j * 8) + i) + ".csv"
                with open(motion, 'rb') as csvinput:
                    writer = csv.writer(csvoutput)

                    for index, row in enumerate(
                            csv.reader(csvinput)):  # index needed by matlab
                        if row:  # remove blank row at the end of files.
                            row = filterFields(row)
                            row.insert(0, index)
                            writer.writerow(row)


if __name__ == "__main__":
    print "main function"

    base = "/home/rafi/Desktop/Level_Lib/"

    motionClass1 = [
        base + "/0/resampled_0.csv",
        base + "/0/resampled_1.csv",
        base + "/0/resampled_2.csv",
        base + "/0/resampled_3.csv",
        base + "/0/resampled_4.csv",
        base + "/0/resampled_5.csv",
        base + "/0/resampled_6.csv",
        base + "/0/resampled_7.csv",
        base + "/0/resampled_8.csv",
        base + "/0/resampled_9.csv",
        base + "/0/resampled_10.csv",
        base + "/0/resampled_11.csv",
        base + "/0/resampled_12.csv",
        base + "/0/resampled_13.csv",
        base + "/0/resampled_14.csv",
        base + "/0/resampled_15.csv",
        base + "/0/resampled_16.csv",
        base + "/0/resampled_17.csv",
        base + "/0/resampled_18.csv",
        base + "/0/resampled_19.csv",
        base + "/0/resampled_20.csv",
        base + "/0/resampled_21.csv",
        base + "/0/resampled_22.csv",
        base + "/0/resampled_23.csv",
        base + "/0/resampled_24.csv",
    ]

    motionClass2 = [
        base + "/1/resampled_0.csv",
        base + "/1/resampled_1.csv",
        base + "/1/resampled_2.csv",
        base + "/1/resampled_3.csv",
        base + "/1/resampled_4.csv",
        base + "/1/resampled_5.csv",
        base + "/1/resampled_6.csv",
        base + "/1/resampled_7.csv",
        base + "/1/resampled_8.csv",
        base + "/1/resampled_9.csv",
        base + "/1/resampled_10.csv",
        base + "/1/resampled_11.csv",
        base + "/1/resampled_12.csv",
        base + "/1/resampled_13.csv",
        base + "/1/resampled_14.csv",
        base + "/1/resampled_15.csv",
        base + "/1/resampled_16.csv",
        base + "/1/resampled_17.csv",
        base + "/1/resampled_18.csv",
        base + "/1/resampled_19.csv",
        base + "/1/resampled_20.csv",
        base + "/1/resampled_21.csv",
        base + "/1/resampled_22.csv",
        base + "/1/resampled_23.csv",
        base + "/1/resampled_24.csv",
    ]

    motionClass3 = [base + "/2/resampled_0.csv",
                    base + "/2/resampled_1.csv",
                    base + "/2/resampled_2.csv",
                    base + "/2/resampled_3.csv",
                    base + "/2/resampled_4.csv",
                    base + "/2/resampled_5.csv",
                    base + "/2/resampled_6.csv",
                    base + "/2/resampled_7.csv",
                    base + "/2/resampled_8.csv",
                    base + "/2/resampled_9.csv",
                    base + "/2/resampled_10.csv",
                    base + "/2/resampled_11.csv",
                    base + "/2/resampled_12.csv",
                    base + "/2/resampled_13.csv",
                    base + "/2/resampled_14.csv",
                    base + "/2/resampled_15.csv",
                    base + "/2/resampled_16.csv",
                    base + "/2/resampled_17.csv",
                    base + "/2/resampled_18.csv",
                    base + "/2/resampled_19.csv",
                    base + "/2/resampled_20.csv",
                    base + "/2/resampled_21.csv",
                    base + "/2/resampled_22.csv",
                    base + "/2/resampled_23.csv",
                    base + "/2/resampled_24.csv",
                    ]

    motionClass4 = [base + "/3/resampled_0.csv",
                    base + "/3/resampled_1.csv",
                    base + "/3/resampled_2.csv",
                    base + "/3/resampled_3.csv",
                    base + "/3/resampled_4.csv",
                    base + "/3/resampled_5.csv",
                    base + "/3/resampled_6.csv",
                    base + "/3/resampled_7.csv",
                    base + "/3/resampled_8.csv",
                    base + "/3/resampled_9.csv",
                    base + "/3/resampled_10.csv",
                    base + "/3/resampled_11.csv",
                    base + "/3/resampled_12.csv",
                    base + "/3/resampled_13.csv",
                    base + "/3/resampled_14.csv",
                    base + "/3/resampled_15.csv",
                    base + "/3/resampled_16.csv",
                    base + "/3/resampled_17.csv",
                    base + "/3/resampled_18.csv",
                    base + "/3/resampled_19.csv",
                    base + "/3/resampled_20.csv",
                    base + "/3/resampled_21.csv",
                    base + "/3/resampled_22.csv",
                    base + "/3/resampled_23.csv",
                    base + "/3/resampled_24.csv",
                    ]

    motionClass5 = [base + "/4/resampled_0.csv",
                    base + "/4/resampled_1.csv",
                    base + "/4/resampled_2.csv",
                    base + "/4/resampled_3.csv",
                    base + "/4/resampled_4.csv",
                    base + "/4/resampled_5.csv",
                    base + "/4/resampled_6.csv",
                    base + "/4/resampled_7.csv",
                    base + "/4/resampled_8.csv",
                    base + "/4/resampled_9.csv",
                    base + "/4/resampled_10.csv",
                    base + "/4/resampled_11.csv",
                    base + "/4/resampled_12.csv",
                    base + "/4/resampled_13.csv",
                    base + "/4/resampled_14.csv",
                    base + "/4/resampled_15.csv",
                    base + "/4/resampled_16.csv",
                    base + "/4/resampled_17.csv",
                    base + "/4/resampled_18.csv",
                    base + "/4/resampled_19.csv",
                    base + "/4/resampled_20.csv",
                    base + "/4/resampled_21.csv",
                    base + "/4/resampled_22.csv",
                    base + "/4/resampled_23.csv",
                    base + "/4/resampled_24.csv",
                    ]

    motionClass6 = [base + "/5/resampled_0.csv",
                    base + "/5/resampled_1.csv",
                    base + "/5/resampled_2.csv",
                    base + "/5/resampled_3.csv",
                    base + "/5/resampled_4.csv",
                    base + "/5/resampled_5.csv",
                    base + "/5/resampled_6.csv",
                    base + "/5/resampled_7.csv",
                    base + "/5/resampled_8.csv",
                    base + "/5/resampled_9.csv",
                    base + "/5/resampled_10.csv",
                    base + "/5/resampled_11.csv",
                    base + "/5/resampled_12.csv",
                    base + "/5/resampled_13.csv",
                    base + "/5/resampled_14.csv",
                    base + "/5/resampled_15.csv",
                    base + "/5/resampled_16.csv",
                    base + "/5/resampled_17.csv",
                    base + "/5/resampled_18.csv",
                    base + "/5/resampled_19.csv",
                    base + "/5/resampled_20.csv",
                    base + "/5/resampled_21.csv",
                    base + "/5/resampled_22.csv",
                    base + "/5/resampled_23.csv",
                    base + "/5/resampled_24.csv",
                    ]

    motionClass7 = [base + "/6/resampled_0.csv",
                    base + "/6/resampled_1.csv",
                    base + "/6/resampled_2.csv",
                    base + "/6/resampled_3.csv",
                    base + "/6/resampled_4.csv",
                    base + "/6/resampled_5.csv",
                    base + "/6/resampled_6.csv",
                    base + "/6/resampled_7.csv",
                    base + "/6/resampled_8.csv",
                    base + "/6/resampled_9.csv",
                    base + "/6/resampled_10.csv",
                    base + "/6/resampled_11.csv",
                    base + "/6/resampled_12.csv",
                    base + "/6/resampled_13.csv",
                    base + "/6/resampled_14.csv",
                    base + "/6/resampled_15.csv",
                    base + "/6/resampled_16.csv",
                    base + "/6/resampled_17.csv",
                    base + "/6/resampled_18.csv",
                    base + "/6/resampled_19.csv",
                    base + "/6/resampled_20.csv",
                    base + "/6/resampled_21.csv",
                    base + "/6/resampled_22.csv",
                    base + "/6/resampled_23.csv",
                    base + "/6/resampled_24.csv",
                    ]

    motionClass8 = [base + "/7/resampled_0.csv",
                    base + "/7/resampled_1.csv",
                    base + "/7/resampled_2.csv",
                    base + "/7/resampled_3.csv",
                    base + "/7/resampled_4.csv",
                    base + "/7/resampled_5.csv",
                    base + "/7/resampled_6.csv",
                    base + "/7/resampled_7.csv",
                    base + "/7/resampled_8.csv",
                    base + "/7/resampled_9.csv",
                    base + "/7/resampled_10.csv",
                    base + "/7/resampled_11.csv",
                    base + "/7/resampled_12.csv",
                    base + "/7/resampled_13.csv",
                    base + "/7/resampled_14.csv",
                    base + "/7/resampled_15.csv",
                    base + "/7/resampled_16.csv",
                    base + "/7/resampled_17.csv",
                    base + "/7/resampled_18.csv",
                    base + "/7/resampled_19.csv",
                    base + "/7/resampled_20.csv",
                    base + "/7/resampled_21.csv",
                    base + "/7/resampled_22.csv",
                    base + "/7/resampled_23.csv",
                    base + "/7/resampled_24.csv",
                    ]

    motionClasses = [motionClass1, motionClass2, motionClass3, motionClass4,
                     motionClass5, motionClass6, motionClass7, motionClass8]

    compound()









