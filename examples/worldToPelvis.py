#!/usr/bin/env python
import os, csv

def toPelvis(Library):
    dir = os.path.dirname(Library[0])
    x0 = None
    y0 = None
    z0 = None

    for i in range(len(Library)):
        compoundIn = dir+'/compound_'+str(i)+'.csv'
        compoundOut = dir+'/pelvis_'+str(i)+'.csv'

        with open(compoundOut, 'wb') as csvoutput:
            with open(compoundIn, 'rb') as csvinput:    
                writer = csv.writer(csvoutput)
                for row in csv.reader(csvinput):
                    if row: #There are empty lines in the csv
                        if row[0] == str(0):
                            x0 = row[1] #PelvisX
                            y0 = row[2] #PelvisY
                            z0 = row[3] #PelvisZ

                            newPelvX = float(row[1]) - float(x0)
                            newPelvY = float(row[2]) - float(y0)
                            newPelvZ = float(row[3]) - float(z0)
                            row[1] = newPelvX
                            row[2] = newPelvY
                            row[3] = newPelvZ
                            writer.writerow(row)
                        else:
                            newPelvX = float(row[1]) - float(x0)
                            newPelvY = float(row[2]) - float(y0)
                            newPelvZ = float(row[3]) - float(z0)
                            row[1] = newPelvX
                            row[2] = newPelvY
                            row[3] = newPelvZ
                            writer.writerow(row)
                        

    
if __name__ == "__main__":
    print "main function"

    Class0 = "/home/rafi/Desktop/Library/compound/compound_0.csv"

    Class1 = "/home/rafi/Desktop/Library/compound/compound_1.csv"

    Class2 = "/home/rafi/Desktop/Library/compound/compound_2.csv"

    Class3 = "/home/rafi/Desktop/Library/compound/compound_3.csv"

    Class4 = "/home/rafi/Desktop/Library/compound/compound_4.csv"

    Class5 = "/home/rafi/Desktop/Library/compound/compound_5.csv"

    Class6 = "/home/rafi/Desktop/Library/compound/compound_6.csv"

    Class7 = "/home/rafi/Desktop/Library/compound/compound_7.csv"


    Library = [Class0, Class1, Class2, Class3, Class4, Class5, Class6, Class7]

    toPelvis(Library)









