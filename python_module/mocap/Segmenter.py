import MocapDrawer
import keystroke
import sys
import SegmentCSV
import MocapCommon
import rospy
import cv2
from os import listdir
from os.path import isfile, join
import os


class Segmenter():

    def __init__(self, m_file, o_file, img_dir, drawer):
        self.m_file = m_file
        self.o_file = o_file
        self.drawer = drawer

        self.drawer.load_file(self.m_file, self.o_file)
        self.max = len(self.drawer.frames)
        self.curr = 0

        self.split = [0,0]
        self.splits = []

        self.images = []
        self.img_dir = img_dir
        self.curr_img = ""

    def change_frame(self, delta):
        self.curr += delta

        if self.curr < 0:
            self.curr = 0
        if self.curr >= self.max:
            print "Reached EOF"
            self.curr = self.max-1


        last = ''

        # Load image
        frame = self.drawer.get_frame(self.curr)
        curr_time = frame.get_time()

        for t, img in self.images:
            if t > curr_time:
                last = img
                break

        self.curr_img = os.path.join(self.img_dir, last)

    def load_images(self):
        images = [ f for f in listdir(self.img_dir) if isfile(join(self.img_dir,f)) and '.png' in f ]
        print "Num images loaded : ", len(images)
        times = []

        for img in images:
            sec, nsec = img.split('_')
            times.append( rospy.Time(int(sec), int(nsec.split('.')[0]) ).to_sec() )

        tuple_list = zip(times, images)
        sorted_images = sorted(tuple_list, key = lambda p : p[0])
        self.images = sorted_images

    def save_splits(self):
        #  Get the out filename
        dir, path = os.path.split(self.m_file)
        split_outpath = dir+'/'+'Splits.csv'

        print "saving splits to : ", split_outpath

        with open(split_outpath, 'a') as s_file:
            for split in self.splits:
                out_str = str(split[0]) + ',' + str(split[1])+'\n'
                s_file.write(out_str)


if __name__ == '__main__':
    base_dir = '/home/rafi/aterm_experiment/'

    setup = MocapCommon.read_setup(base_dir)
    NB_HUMAN    = setup[0]
    ELBOW_PADS  = setup[1]
    RARM_ONLY   = setup[2]
    NB_MARKERS  = MocapCommon.get_nb_markers(ELBOW_PADS, RARM_ONLY)

    d =  MocapDrawer.Drawer(NB_MARKERS, NB_HUMAN, ELBOW_PADS, RARM_ONLY)

    blocks = sorted([ name for name in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, name)) ])
    for block in blocks:
        path = os.path.join(base_dir, block)
        runs = sorted([ name for name in os.listdir(path) if os.path.isdir(os.path.join(path, name)) ])

        for run in runs:
            file_path = os.path.join(path, run)
            print
            print 'Trying to fix block : ', block, ' run : ', run
            print 'in folder : ', file_path
            print'================================================'
            print
            m_file = os.path.join(file_path,'markers_fixed.csv')
            o_file = os.path.join(file_path,'objects_fixed.csv')

            s = Segmenter(m_file, o_file, file_path, d)
            s.load_images()

            while True:
                c = keystroke.getch(-1)

                if c == 'q':
                    sys.exit()
                if c == 'u':
                    s.change_frame(-25)
                if c == 'i':
                    s.change_frame(-1)
                if c == 'o':
                    s.change_frame(1)
                if c == 'p':
                    s.change_frame(25)
                if c == ' ':
                    print "Current Frame: ", s.curr
                    print "Current Splits : ", s.splits
                if c == '1':
                    s.split[0] = s.curr
                    print "Set split beginning to: " + str(s.split[0])
                    continue
                if c == '2':
                    s.split[1] = s.curr
                    print "Set split ending to: " + str(s.split[1])
                    continue
                if c == 's':
                    print "Segmenting files from: " + str(s.split[0]) + " to: " + str(s.split[1])

                    s.splits.append((s.split[0], s.split[1]))
                    print "Current splits : "
                    print s.splits

                    # temp = SegmentCSV.Segmenter(s.m_file, './markers.csv')
                    # temp.segment([(s.split[0], s.split[1])])
                    # temp = SegmentCSV.Segmenter(s.o_file, './objects.csv')
                    # temp.segment([(s.split[0], s.split[1])])
                if c == 'r':
                    print "Removing split : ", s.splits.pop()
                    continue
                if c == 'x':
                    if len(s.splits) > 0:
                        s.save_splits()
                    # Reset all splits
                    s.splits = []
                    print "Saved splits to Splits.csv, clearing splits"
                    break

                # TODO Should only be done if we updated a frame.  Get rid of continues
                cv_image = cv2.imread( s.curr_img, 1)
                if cv_image is not None:
                    cv2.imshow('Camera Data', cv_image)

                s.drawer.clear()
                frame = s.drawer.get_frame(s.curr)
                s.drawer.draw_frame_skeleton(frame)




