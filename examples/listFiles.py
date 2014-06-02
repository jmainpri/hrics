import os

for root, dirs, filenames in os.walk("/home/rafi/workspace/statFiles/recorded_motion/"):
    for f in filenames:
        print "\""+f+"\","
