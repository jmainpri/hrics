#!/usr/bin/python

# Rafi Hayne

import os
import csv

class Segmenter:

    def __init__(self, infile, outfile):
        self.fin = infile
        self.fout = outfile


    # splits is a list of tuples [ (start, end), ... ]
    def segment(self, splits):

        path, file = os.path.split(self.fout)

        for split in splits:
            with open(self.fin, 'r') as csv_in:
                start = split[0]
                end = split[1]

                if start < end:
                    out_str = path+ '/' + '['+ str( start ).zfill(4) + '-' + str( end ).zfill(4) + ']' + file

                    with open(out_str, 'w') as csv_out:
                        w = csv.writer(csv_out)
                        r = csv.reader(csv_in)
                        for row in list(r)[start:end]:
                            w.writerow(row)

                else:
                    print str(start), " isn't < ", str(end), " no split possible"

