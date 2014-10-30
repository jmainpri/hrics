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


class Human():

    def __init__(self, marker_list, object_list, elbow_pads, r_arm_only):
        self.markers = marker_list
        self.objects = object_list
        self.elbow_pads = elbow_pads
        self.r_arm_only = r_arm_only

    def get_center_points(self):
        # Get the center points
        Chest = (self.markers[0].array + self.markers[1].array)/2
        Sternum = (self.markers[2].array + self.markers[3].array)/2
        rShoulder = (self.markers[4].array + self.markers[5].array)/2
        if self.elbow_pads:
            rElbow = self.objects[2].array
            rWrist = (self.markers[6].array + self.markers[7].array)/2
            rPalm = self.markers[8].array
            if not self.r_arm_only:
                lShoulder = (self.markers[9].array + self.markers[10].array)/2
                lElbow = self.objects[3].array
                lWrist = (self.markers[11].array + self.markers[12].array)/2
                lPalm = self.markers[13].array
        else:
            rElbow = (self.markers[6].array + self.markers[7].array)/2
            rWrist = (self.markers[8].array + self.markers[9].array)/2
            rPalm = self.markers[10].array
            if not self.r_arm_only:
                lShoulder = (self.markers[11].array + self.markers[12].array)/2
                lElbow = (self.markers[13].array + self.markers[14].array)/2
                lWrist = (self.markers[15].array + self.markers[16].array)/2
                lPalm = self.markers[17].array

        Pelvis = self.objects[0].array
        Head = self.objects[1].array

        # Return the points
        centers = [Chest, Sternum, rShoulder, rElbow, rWrist, rPalm]
        if not self.r_arm_only:
            lArm = [lShoulder, lElbow, lWrist, lPalm]
            centers += lArm
        centers += [Pelvis, Head]

        return np.array(centers)

class Drawer():

    def __init__(self, nb_markers, nb_human, elbow_pads, r_arm_only):

        self.nb_human = nb_human
        self.elbow_pads = elbow_pads
        self.r_arm_only = r_arm_only
        self.nb_markers = nb_markers
        self.nb_objects = get_nb_objects(self.elbow_pads, self.r_arm_only)

        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.Reset()
        # self.env.Load("../../ormodels/human_wpi_bio.xml")

        self.handles = []

        self.frames = []

        t_cam = array([[ -0.655253290114, -0.106306078558, 0.747891799297, -0.302201271057] , \
                                [ -0.725788890663, 0.363116971923, -0.584274379801, 2.68592453003] , \
                                [ -0.209460287369, -0.925659269042, -0.315089361376, 2.25037527084] , \
                                [ 0.0, 0.0, 0.0, 1.0]])
        self.env.GetViewer().SetCamera(t_cam)

    def load_file(self, m_filepath, o_filepath):
        print "Trying to open file"
        # global NB_HUMAN # TODO fix global to be class member

        marker_file = []
        object_file = []

        with open(m_filepath, 'r') as m_file:
            with open(o_filepath, 'r') as o_file:

                marker_file = [row for row in csv.reader(m_file, delimiter=',')]
                object_file = [row for row in csv.reader(o_file, delimiter=',')]

        nb_lines = min(len(marker_file), len(object_file))
        self.last_frame = nb_lines

        for row in range(nb_lines):

            markers = []
            objects = []

            m_cells = marker_file[row]
            o_cells = object_file[row]

            # Load Objects
            count = int(o_cells[2])

            # Assuming only using Pelv/Head objects per person and nothing else in the scene
            # NB_HUMAN = count/2

            for i in range(3, count*9, 9):
                name = str(o_cells[i])
                occluded = int(o_cells[i+1])
                x = float(o_cells[i+2])
                y = float(o_cells[i+3])
                z = float(o_cells[i+4])
                r_x = float(o_cells[i+5])
                r_y = float(o_cells[i+6])
                r_z = float(o_cells[i+7])
                r_w = float(o_cells[i+8])

                object = Object( name, occluded, x, y, z, r_x, r_y, r_z, r_w )
                objects.append(object)

            # Load Markers
            sec = float(m_cells[0])
            nsec = float(m_cells[1])
            count = int(m_cells[2])

            for i in range(3, count*4, 4):
                id = m_cells[i]
                x = float(m_cells[i+1])
                y = float(m_cells[i+2])
                z = float(m_cells[i+3])

                marker = Marker(id, x, y, z)
                markers.append(marker)


            self.frames.append( Frame(sec, nsec, count, markers, objects) )

    # Split is a tuple of the form (start_frame, end_frame)
    def load_range(self, m_filepath, o_filepath, split):
        print "Trying to open file"
        # global NB_HUMAN # TODO fix global to be class member

        marker_file = []
        object_file = []

        with open(m_filepath, 'r') as m_file:
            with open(o_filepath, 'r') as o_file:

                marker_file = [row for row in csv.reader(m_file, delimiter=',')]
                object_file = [row for row in csv.reader(o_file, delimiter=',')]

        nb_lines = min(len(marker_file), len(object_file))
        self.last_frame = nb_lines

        for row in range(split[0], split[1]):

            markers = []
            objects = []

            m_cells = marker_file[row]
            o_cells = object_file[row]

            # Load Objects
            count = int(o_cells[2])

            # Assuming only using Pelv/Head objects per person and nothing else in the scene
            # NB_HUMAN = count/2

            for i in range(3, count*9, 9):
                name = str(o_cells[i])
                occluded = int(o_cells[i+1])
                x = float(o_cells[i+2])
                y = float(o_cells[i+3])
                z = float(o_cells[i+4])
                r_x = float(o_cells[i+5])
                r_y = float(o_cells[i+6])
                r_z = float(o_cells[i+7])
                r_w = float(o_cells[i+8])

                object = Object( name, occluded, x, y, z, r_x, r_y, r_z, r_w )
                objects.append(object)

            # Load Markers
            sec = float(m_cells[0])
            nsec = float(m_cells[1])
            count = int(m_cells[2])

            for i in range(3, count*4, 4):
                id = m_cells[i]
                x = float(m_cells[i+1])
                y = float(m_cells[i+2])
                z = float(m_cells[i+3])

                marker = Marker(id, x, y, z)
                markers.append(marker)


            self.frames.append( Frame(sec, nsec, count, markers, objects) )

    def play_raw(self):
        # for frame in self.frames:
        prev_time = self.frames[0].get_time()

        for i, frame in enumerate(self.frames):
            curr_time = frame.get_time()
            # self.draw_point_by_name(frame, 'Chest')
            self.draw_frame_raw(frame)
            self.draw_frame_axes(frame)

            dt = curr_time - prev_time
            prev_time = curr_time
            time.sleep(dt)
            self.clear()


        return

    def play_skeleton(self):
        # for frame in self.frames:
        prev_time = self.frames[0].get_time()

        # IMAGE PUBLISHER
        folder = "/home/rafi/logging_ten/1"
        images = [ f for f in listdir(folder) if isfile(join(folder,f)) and '.png' in f ]
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


        for i, frame in enumerate(self.frames):
            curr_time = frame.get_time()

            self.draw_frame_skeleton(frame)

            dt = curr_time - prev_time
            # dt = 0.05
            prev_time = curr_time
            time.sleep(dt)

            if True:
            # if i % 20 == 0 :
                last = None

                try:
                    for t, img in sorted_images:
                        if t > curr_time:
                            # image_name = folder + "/" + img
                            break

                        last = img

                    if last:
                        image_name = folder + "/" + last
                        print image_name

                        cv_image = cv2.imread( image_name, 1)
                        cv2.imshow('Camera Data', cv_image)
                        # print image_name
                except e:
                    print e


                if i == 0:
                    sys.stdin.readline()

            #     # print "blocking?"
            #     # rospy.spin()

                # print "Frame : ", i , " press enter"

            self.clear()

        return

    def isolate_humans(self, frame):
        humans = []

        for i in range(self.nb_human):
            # Ugly ;(
            temp_markers = frame.marker_list[i*self.nb_markers:i*self.nb_markers+self.nb_markers]
            temp_objects = frame.object_list[i*self.nb_objects:i*self.nb_objects+self.nb_objects]

            humans.append( Human(temp_markers, temp_objects, self.elbow_pads, self.r_arm_only) )

        return humans

    def get_frame(self, frame_id):
        return self.frames[frame_id]

    def draw_frame_skeleton(self, frame):
        humans = self.isolate_humans(frame)
        # self.draw_frame_axes(frame)
        self.draw_frame_raw(frame)

        for h in humans:
            centers = h.get_center_points()
            self.draw_skeleton(centers)

    def draw_frame_raw(self, frame):
        point_list = []

        for m in frame.marker_list:
            point_list.append(m.array)
        for o in frame.object_list:
            if o:
                point_list.append(o.array)

        self.draw_points(np.array(point_list))

    def draw_point_by_name(self, frame, name):
        point_list = []

        for m in frame.marker_list:
            if name in m.id:
                point_list.append(m.array)

        self.draw_point_isolated(np.array(point_list))

    def draw_point_isolated(self, point_list):
        points = point_list

        colors = []
        nb_points = len(points)
        for n in range(len(point_list)):
            if n == 17:
                colors.append((1,0,0))
            else:
                colors.append((0,0,1))

        self.handles.append(self.env.plot3(points=point_list, pointsize=0.05, colors=array(colors), drawstyle=1))


    def draw_points(self, point_list):
        points = point_list

        colors = []
        nb_points = len(points)
        for n in linspace(0.0, 1.0, num=nb_points):
            # colors.append((0,0,1))
            colors.append((float(n)*1, (1-float(n))*1, 0))

        self.handles.append(self.env.plot3(points=point_list, pointsize=0.02, colors=array(colors), drawstyle=1))


    def draw_skeleton(self, point_list):
        points = point_list

        colors = []
        nb_points = len(points)
        for n in linspace(0.0, 1.0, num=nb_points):
            colors.append((float(n)*1, (1-float(n))*1, 0))

        # Marker points
        self.handles.append(self.env.plot3(points=point_list, pointsize=0.02, colors=array(colors), drawstyle=1))

        if not self.r_arm_only:

            # Connect shoulders
            shoulders = np.array([point_list[2],point_list[6]])
            self.handles.append(self.env.drawlinestrip(points=shoulders, linewidth=3.0))

            # Connect head
            head = np.array( [ point_list[1], point_list[11] ] )
            self.handles.append(self.env.drawlinestrip(points=head, linewidth=3.0))

            # Connect right arm
            right_arm = np.array([point_list[2],point_list[3], point_list[4], point_list[5]])
            self.handles.append(self.env.drawlinestrip(points=right_arm, linewidth=3.0))

            # Left Arm
            left_arm = np.array( [point_list[6], point_list[7], point_list[8], point_list[9]] )
            self.handles.append(self.env.drawlinestrip(points=left_arm, linewidth=3.0))

            # Pelvis
            pelv = np.array( [point_list[2], point_list[10], point_list[6]] )
            self.handles.append(self.env.drawlinestrip(points=pelv, linewidth=3.0))

        else:
            # Connect Shoulders
            shoulders = np.array([point_list[2],point_list[1]])
            self.handles.append(self.env.drawlinestrip(points=shoulders, linewidth=3.0))

            # Connect head
            head = np.array( [ point_list[1], point_list[7] ] )
            self.handles.append(self.env.drawlinestrip(points=head, linewidth=3.0))

            # Connect right arm
            right_arm = np.array([point_list[2],point_list[3], point_list[4], point_list[5]])
            self.handles.append(self.env.drawlinestrip(points=right_arm, linewidth=3.0))

            # Pelvis
            pelv = np.array( [point_list[2], point_list[1], point_list[6], point_list[2]] )
            self.handles.append(self.env.drawlinestrip(points=pelv, linewidth=3.0))

    def draw_object_axes(self, object):
        # tf = MakeTransform( rotationMatrixFromQuat( array(transformation_helper.NormalizeQuaternion([object.r_w, object.r_x, object.r_y, object.r_z]) )), transpose(matrix([object.x, object.y, object.z])) )

        tf = object.get_transform()
        self.handles.append(misc.DrawAxes( self.env, matrix(tf), 0.5 ))

    def draw_frame_axes(self, frame):
        for o in frame.object_list:
            if not o.is_occluded():
                self.draw_object_axes(o)

    def clear(self):
        del self.handles[:]


if __name__ == '__main__':

    setup = read_setup('/home/rafi/Desktop/trials/')
    # splits = read_splits('/home/rafi/logging_ten/0/')


    # NB_HUMAN    = setup[0]
    # ELBOW_PADS  = setup[1]
    # RARM_ONLY   = setup[2]
    # NB_MARKERS = get_nb_markers(ELBOW_PADS, RARM_ONLY)

    # NB_HUMAN    = setup[0]
    NB_HUMAN    = 1
    ELBOW_PADS  = False
    RARM_ONLY   = False
    NB_MARKERS = get_nb_markers(ELBOW_PADS, RARM_ONLY)

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/markers_fixed.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/objects_fixed.csv'

    # m_file = '/home/rafi/Desktop/TEMP/[0580-0680]markers.csv'
    # o_file = '/home/rafi/Desktop/TEMP/[0580-0680]objects.csv'

    # m_file = '/home/rafi/Desktop/TEMP/[1300-1420]markers.csv'
    # o_file = '/home/rafi/Desktop/TEMP/[1300-1420]objects.csv'

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[1000-3900]markers_fixed.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[1000-3900]objects_fixed.csv'

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[5900-9000]markers_fixed.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[5900-9000]objects_fixed.csv'

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[22400-25300]markers_fixed.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[22400-25300]objects_fixed.csv'

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[28300-30800]markers_fixed.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[28300-30800]objects_fixed.csv'

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[33000-35700]markers_fixed.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[33000-35700]objects_fixed.csv'

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[37900-40400]markers_fixed.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[37900-40400]objects_fixed.csv'

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[42600-44700]markers_fixed.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[42600-44700]objects_fixed.csv'

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/markers_fixed.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/objects_fixed.csv'

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/markers_fixed.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/objects_fixed.csv'

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/markers_fixed.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/objects_fixed.csv'

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/markers_fixed.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/objects_fixed.csv'

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/markers_fixed.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/objects_fixed.csv'


    dirr = '/home/rafi/Desktop/trials/'

    # Recording 9 Run 1
    # run = '1/'
    # prefix = '[0446-0578]'
    # prefix = '[0780-0871]'
    # prefix = '[2554-2671]'

    # Recording 9 Run 2
    # run = '2/'
    # prefix = '[0525-0657]'
    # prefix = '[2197-2343]'
    # prefix = '[2711-2823]'

    # Recording 9 Run 3
    # run = '3/'
    # prefix = '[0444-0585]'
    # prefix = '[1064-1140]'
    # prefix = '[1342-1451]'
    # prefix = '[1882-1981]'
    # prefix = '[2172-2249]'
    # prefix = '[2646-2737]'

    # # Recording 9 Run 4
    run = '4/'
    prefix = '[0489-0589]'
    # prefix = '[1537-1608]'
    # prefix = '[2018-2099]'

    # Recording 9 Run 5
    # Empty

    # Recording 9 Run 6
    # run = '6/'
    # prefix = '[0408-0491]'
    # prefix = '[0889-0945]'
    # prefix = '[1188-1256]'




    m_file = dirr+run+prefix+'markers.csv'
    o_file = dirr+run+prefix+'objects.csv'


    # f = MarkerFixer('/home/rafi/logging_nine/2/[5900-9000]markers.csv', '/home/rafi/logging_nine/2/[5900-9000]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[11700-14800]markers.csv', '/home/rafi/logging_nine/2/[11700-14800]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[22400-25300]markers.csv', '/home/rafi/logging_nine/2/[22400-25300]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[28300-30800]markers.csv', '/home/rafi/logging_nine/2/[28300-30800]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[33000-35700]markers.csv', '/home/rafi/logging_nine/2/[33000-35700]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[37900-40400]markers.csv', '/home/rafi/logging_nine/2/[37900-40400]objects.csv')
    # f = MarkerFixer('/home/rafi/logging_nine/2/[42600-44700]markers.csv', '/home/rafi/logging_nine/2/[42600-44700]objects.csv')

    # m_file = '/home/rafi/logging_ten/0/markers_fixed.csv'
    # o_file = '/home/rafi/logging_ten/0/objects_fixed.csv'

    folder = '/home/rafi/logging_eleven/0/'
    # name = '[0001-3406]'
    # name = '[8657-11482]'
    # name = '[13712-17083]'
    name = '[19458-22078]'
    # name = '[24495-27070]'

    m_file = folder + name + 'markers_fixed.csv'
    o_file = folder + name + 'objects_fixed.csv'

    # m_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[0629-0768]markers.csv'
    # o_file = '/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[0629-0768]objects.csv'

    # block = '1'
    # run = '0'
    # m_file = '/home/rafi/aterm_experiment/block'+str(block)+'/'+str(run)+'/markers_fixed.csv'
    # o_file = '/home/rafi/aterm_experiment/block'+str(block)+'/'+str(run)+'/objects_fixed.csv'

    m_file = '/home/rafi/two_arm_test_data/[0800-3511]markers.csv'
    o_file = '/home/rafi/two_arm_test_data/[0800-3511]objects.csv'



    d =  Drawer(NB_MARKERS, NB_HUMAN, ELBOW_PADS, RARM_ONLY)
    d.load_file(m_file, o_file)
    d.play_skeleton()

    # for split in splits:
    #     print "Playing split : ", split
    #     d.load_range(m_file, o_file, split)
    #     d.play_skeleton()
    #     print "Finished playing split.  Press enter to continue"
    #     sys.stdin.readline()
