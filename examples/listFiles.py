import os

for root, dirs, filenames in os.walk("/home/rafihayne/workspace/statFiles/recorded_motion/"):
    for f in filenames:
        print "\""+f+"\","
