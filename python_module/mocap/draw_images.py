#!/usr/bin/python

# Rafi Hayne

from openravepy import *
from MocapCommon import *
import transformation_helper
# from bioik.BioHumanIk import *
import csv
import sys


# TMP ROS STUFF
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from os import listdir
from os.path import isfile, join


class Images():

    def __init__(self):

        print "init node"
        rospy.init_node('images_drawer')

        self.folder = None
        self.print_time = False


    def play(self):

        # IMAGE PUBLISHER
        images = [f for f in listdir(self.folder) if isfile(join(self.folder, f)) and '.png' in f]
        print "Num images : ", len(images)
        times = []

        for img in images:
            sec, nsec = img.split('_')
            times.append( rospy.Time(int(sec), int(nsec.split('.')[0]) ).to_sec() )

        tuple_list = zip(times, images)
        sorted_images = sorted(tuple_list, key = lambda p : p[0])

        # print sorted_images
        # Realtime images playback
        # READ IMAGES
        # t_old = None
        # for t, img in sorted_images:
        #     sec, nsec = img.split('_')
        #     cv_image = cv2.imread( folder + "/" + img, 1)
        #     cv2.imshow('my_picture',cv_image)
        #     if t_old is None : t_old = t
        #     time.sleep(t-t_old)
        #     t_old = t


        # # Step by step image view
        # for i, (t, img) in enumerate(sorted_images):
        #     if i % 100 == 0:
        #         print i
        #         sec, nsec = img.split('_')
        #         cv_image = cv2.imread( folder + "/" + img, 1)
        #         cv2.imshow('my_picture',cv_image)
        #         sys.stdin.readline()

        while not rospy.is_shutdown():

            time.sleep(0.01)
            last = None
            curr_time = rospy.get_time()
            
            if self.print_time:
                print "time : ", curr_time

            try:

                for t, img in sorted_images:

                    if t > curr_time:
                        # image_name = folder + "/" + img
                        break

                    last = img

                if last:

                    image_name = self.folder + "/" + last
                    # print image_name	

                    cv_image = cv2.imread(image_name, 1)
                    cv2.imshow('Camera Data', cv_image)
                    cv2.waitKey(3)
                    # print image_name
            except e:
                print e

        return

if __name__ == "__main__":

    imgs = Images()

    print "len(sys.argv) : ", len(sys.argv)

    if len(sys.argv) > 1:

        for index in range(1, len(sys.argv)):

            if sys.argv[index] == "-d" and index+1 < len(sys.argv):
                imgs.folder = str(sys.argv[index+1])

            elif sys.argv[index] == "-t":
                imgs.print_time = True

        print "Images server started, waiting for queries"
        imgs.play()

    else:

        print "usage: ./draw_images.py -d directory"

