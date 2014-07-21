#!/usr/bin/python

# Rafi Hayne

import openravepy
from TransformMatrix import *
from get_marker_names import *
import csv
import numpy as np
import os
import collections
import time
import math
import sys
import copy

THRESHOLD   = 0.0025
NB_MARKERS  = 18
NB_HUMAN    = 2

#Just for timing runs of the tracker.  Useless
class Timer:
    def __enter__(self):
        self.start = time.clock()
        return self

    def __exit__(self, *args):
        self.end = time.clock()
        self.interval = self.end - self.start


class Marker:
    def __init__(self, id, x, y, z, name=''):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.name = name
        self.array = np.array([self.x, self.y, self.z])

    def is_same_marker(self, a_marker, threshold):
        if self.get_dist(a_marker) > threshold:
            return False
        else:
            return True

    def numpy(self):
        return self.array

    def get_dist(self, other):
        diff = self.array - other.array
        return np.dot(diff, diff.conj())


class Object:
    def __init__(self, id, occluded, x, y, z, r_x, r_y, r_z, r_w):
        self.id = id
        self.occluded = occluded
        self.x = x
        self.y = y
        self.z = z
        self.r_x = r_x
        self.r_y = r_y
        self.r_z = r_z
        self.r_w = r_w

    def is_occluded(self):
        return self.occluded

    def get_rot_matrix(self):
        mat =  MakeTransform( openravepy.rotationMatrixFromQuat(np.array([self.r_x, self.r_y, self.r_z, self.r_w])), np.transpose(np.matrix([self.x, self.y, self.z])))

        if self.id is 'TouchTomorrow3':
            x_dir = np.array(np.transpose(mat[:,0]).tolist()[0][:3])
            y_dir = np.array(np.transpose(mat[:,1]).tolist()[0][:3])
            z_dir = np.array(np.transpose(mat[:,2]).tolist()[0][:3])

            new_x = -z_dir
            new_x[2] = 0
            new_x = new_x/np.linalg.norm(new_x)
            new_z = np.array([0,0,1])

            new_y = np.cross(new_x, new_z)
            new_y = new_y/np.linalg.norm(new_y)
            new_y = -new_y


            mat[0][0, 0] = new_x[0]
            mat[0][0, 1] = new_y[0]
            mat[0][0, 2] = new_z[0]
            mat[1][0, 0] = new_x[1]
            mat[1][0, 1] = new_y[1]
            mat[1][0, 2] = new_z[1]
            mat[2][0, 0] = new_x[2]
            mat[2][0, 1] = new_y[2]
            mat[2][0, 2] = new_z[2]

        return mat


class Frame:
    def __init__(self, t_sec, t_usec, nb_markers, marker_list, object_list):
        self.sec = t_sec
        self.usec = t_usec
        self.count = nb_markers
        self.marker_list = marker_list
        self.object_list = object_list

    # Can not have any duplicate ids
    def order_markers(self):
        self.marker_list.sort(key=lambda m: m.id, reverse=False)
        # self.print_marker_ids()
        # new_list = [None]*(self.count+1)
        # for marker in self.marker_list:
        #     print "id : ", marker.id, "  len : ", len(self.marker_list), ' count : ', self.count
        #     new_list[marker.id] = marker
        #
        # self.marker_list = new_list

    def get_marker_by_id(self, id):
        for marker in self.marker_list:
            if marker.id == id:
                return marker


        print "Couldn't find marker with id : ", id
        return None

    def print_marker_ids(self):
        marker_string = '[ '

        for marker in self.marker_list:
            if marker is None:
                marker_string += 'None' + ' '
            else:
                marker_string += str(marker.id) + ' '
        marker_string += ']'
        print marker_string

    def is_similar_frame(self, other):
        if self.count != other.count or self.count != N_MARKERS:
            return False

        for i in range(self.count):

            if self.marker_list[i] is None:
                return False

            if not self.marker_list[i].is_same_marker(other.marker_list[i], THRESHOLD):
                return False

        return True

    # Returns a list of tuples sorted by lowest value (distance)
    # [ ( key, val), ... etc
    def get_distances(self, other):
        dist = {}
        for marker in self.marker_list:
            # dist[marker.id] = random.random()
            dist[marker.id] = marker.get_dist(other)
        return sorted(dist.items(), key=lambda(k, v): v)

    def get_index_list_by_id(self, id):
        index_list = []

        for i,marker in enumerate(self.marker_list):
            if marker.id == id:
                index_list.append(i)

        return index_list

    def get_unused_id(self):
        # unused = []
        # for i in range(0, len(self.marker_list)):
        #     unused.append(i)
        # for marker in self.marker_list:
        #     if marker.id in unused:
        #         unused.remove(marker.id)

        unused = []
        for i, marker in enumerate(self.marker_list):
            if marker == None:
                unused.append(i)

        return unused

    # def interpolate(self, configs, curr_id, id_missing):
    #     prev = configs[curr_id-1]
    #     next = configs[curr_id+1]
    #
    #     m_prev = prev.get_marker_by_id(id_missing)
    #     print next.get_distances(m_prev)[0][0]
    #     if m_prev != None:
    #         m_next = next.get_marker_by_id(next.get_distances(m_prev)[0][0])
    #
    #     x = m_prev.x+(m_next.x-m_prev.x)*0.5
    #     y = m_prev.y+(m_next.y-m_prev.y)*0.5
    #     z = m_prev.z+(m_next.z-m_prev.z)*0.5
    #
    #     return Marker(m_prev.id, x, y, z)


    def get_duplicate_ids(self):
        indices = []

        for marker in self.marker_list:
            if not marker == None:
                indices.append(marker.id)

        return [x for x, y in collections.Counter(indices).items() if y > 1]

    def get_new_config_by_distance(self, prev_frame):
        # print "i : ", prev_index+1
        new_markers = [None]*prev_frame.count
        shortest_found = [float('inf')]*prev_frame.count

        # self.print_marker_ids()

        for i, marker in enumerate(self.marker_list):

            dists = prev_frame.get_distances(marker)

            for closest in dists:
                closest_id = closest[0]
                closest_dist = closest[1]

                if closest_id < prev_frame.count and shortest_found[closest_id] > closest_dist:
                    new_markers[closest_id] = Marker(closest_id, marker.x, marker.y, marker.z, prev_frame.marker_list[closest_id].name)
                    shortest_found[closest_id] = closest_dist
                    break

        for i in range(0, prev_frame.count):
            if new_markers[i] is None:  # Marker dropped!
                print "Dropped a marker"
                prev_frame.print_marker_ids()
                new_markers[i] = prev_frame.get_marker_by_id(i)

        # for i,marker in enumerate(new_markers):
        #     if marker == None:
        #         new_markers.pop(i)

        # print "frame : ", prev_index+1
        temp = Frame(self.sec, self.usec, len(new_markers), new_markers, self.object_list)

        return temp

    def set_marker_names_by_id(self):
        for marker in self.marker_list:
            if marker.id == 0:
                marker.name = 'ChestFront'
            if marker.id == 1:
                marker.name = 'ChestBack'
            if marker.id == 2:
                marker.name = 'SternumFront'
            if marker.id == 3:
                marker.name = 'SternumBack'
            if marker.id == 4:
                marker.name = 'rShoulderFront'
            if marker.id == 5:
                marker.name = 'rShoulderBack'
            if marker.id == 6:
                marker.name = 'rElbowOuter'
            if marker.id == 7:
                marker.name = 'rElbowInner'
            if marker.id == 8:
                marker.name = 'rWristOuter'
            if marker.id == 9:
                marker.name = 'rWristInner'
            if marker.id == 10:
                marker.name = 'rPalm'
            if marker.id == 11:
                marker.name = 'lShoulderFront'
            if marker.id == 12:
                marker.name = 'lShoulderBack'
            if marker.id == 13:
                marker.name = 'lElbowOuter'
            if marker.id == 14:
                marker.name = 'lElbowInner'
            if marker.id == 15:
                marker.name = 'lWristOuter'
            if marker.id == 16:
                marker.name = 'lWristInner'
            if marker.id == 17:
                marker.name = 'lPalm'

    def set_marker_names(self):
        true_count = NB_MARKERS*NB_HUMAN
        if (self.count is not true_count):
            print "Something went wrong when setting marker names"
            return

        new_id = 0
        for i in range(0, true_count, NB_MARKERS):
            self.marker_list[i].name = 'ChestFront'
            self.marker_list[i+1].name = 'ChestBack'
            self.marker_list[i+2].name = 'SternumFront'
            self.marker_list[i+3].name = 'SternumBack'
            self.marker_list[i+4].name = 'rShoulderFront'
            self.marker_list[i+5].name = 'rShoulderBack'
            self.marker_list[i+6].name = 'rElbowOuter'
            self.marker_list[i+7].name = 'rElbowInner'
            self.marker_list[i+8].name = 'rWristOuter'
            self.marker_list[i+9].name = 'rWristInner'
            self.marker_list[i+10].name = 'rPalm'
            self.marker_list[i+11].name = 'lShoulderFront'
            self.marker_list[i+12].name = 'lShoulderBack'
            self.marker_list[i+13].name = 'lElbowOuter'
            self.marker_list[i+14].name = 'lElbowInner'
            self.marker_list[i+15].name = 'lWristOuter'
            self.marker_list[i+16].name = 'lWristInner'
            self.marker_list[i+17].name = 'lPalm'


    # def get_n_closest_markers(self, pelv_frame, n):
    #     points = []
    #     points_in_pelv = {}
    #     temp_pelv = np.transpose(pelv_frame[:,3])
    #     pelv_point = np.array([temp_pelv[0,0], temp_pelv[0,1], temp_pelv[0,2]])
    #     # pelv_marker = Marker( 0, pelv_point[0,0], pelv_point[0,1], pelv_point[0,2]  )

    #     # Get all points in the pelvis frame
    #     inv = np.linalg.inv(pelv_frame)
    #     for marker in self.marker_list:

    #         point = np.array(np.array(inv).dot(np.array(append(marker.array, 1.0))))[0:3]
    #         point[2] = pelv_point[2]  # Only consider closest in x and y

    #         points_in_pelv[marker.id] = point

    #     # Get their distance from the pelvis point
    #     dist = {}
    #     for key in points_in_pelv:
    #         dist[key] = np.linalg.norm(points_in_pelv[key]-pelv_point)

    #     dist_list = sorted(dist.items(), key=lambda(k, v): v)
    #     # Get the first 18 points
    #     for pair in dist_list[:n]:
    #         points.append( ( pair[0] ,self.marker_list[pair[0]].array) )

    #     return points

    def get_n_closest_markers(self, pelv_frame, n):
        points = []
        dist = {}

        for marker in self.marker_list:
            # Only consider distance in x and y
            dist[marker.id] = sqrt( (marker.x-pelv[0])**2 + (marker.y-pelv[1])**2 )

        dist_list = sorted(dist.items(), key=lambda(k, v): v)


        # Return the first n points
        for pair in dist_list[:n]:
            points.append( ( pair[0] ,self.marker_list[pair[0]].array) )

        return points


class Fixer:

    def __init__(self, m_filepath, o_filepath):
        self.m_filepath = m_filepath
        self.o_filepath = o_filepath
        self.frames = []
        self.max_markers = None
        self.first_frame = 0
        self.last_frame = 0

    def load_file(self):
        print "Trying to open file"

        marker_file = []
        object_file = []

        with open(self.m_filepath, 'r') as m_file:
            with open(self.o_filepath, 'r') as o_file:

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
                id = int(m_cells[i])
                x = float(m_cells[i+1])
                y = float(m_cells[i+2])
                z = float(m_cells[i+3])

                marker = Marker(id, x, y, z)
                markers.append(marker)


            self.frames.append( Frame(sec, nsec, count, markers, objects) )

        print "# configs loaded : " + str(len(self.frames))

    def save_file(self):
        print "Trying to output new file"

        #  Get the out filename
        dir, path = os.path.split(self.m_filepath)
        name, type = path.rsplit('.', 1)
        outpath = name + '_fixed.'+type


        # TODO Should output a truncated objects file starting at first_frame
        with open(outpath, 'w') as f:
            for frame in self.frames[self.first_frame:self.last_frame]:
                line_str = ""
                line_str += str(frame.sec) + ','
                line_str += str(frame.usec) + ','
                line_str += str(frame.count) + ','

                for marker in frame.marker_list:
                    line_str += str(marker.id) + ','
                    line_str += str(marker.x) + ','
                    line_str += str(marker.y) + ','
                    line_str += str(marker.z) + ','

                line_str = line_str.rstrip(',')
                line_str += '\n'
                f.write(line_str)

    def get_average_position(self):
        m_tot = np.array( [0, 0, 0] )
        m_count = 0.0

        for frame in self.frames:
            for marker in frame.marker_list:
                m_tot += marker.numpy()
                m_count += 1

        print "trying to finding avg pos"

        return m_tot/m_count

    def filter_threshold_inside(self, point, threshold):
        nb_removed = 0
        avg = Marker(0, point[0], point[1], point[2])

        print "Trying to filter markers ", threshold, "m within : ", point
        for frame in self.frames:

            remove_list = []

            for marker in frame.marker_list:
                if math.sqrt(avg.get_dist(marker)) < threshold:
                    remove_list.append(marker)
                    frame.count -= 1
                    nb_removed += 1

            for r_marker in remove_list:
                frame.marker_list.remove(r_marker)

        print "Filtered ", nb_removed, ' markers'

    def filter_pillar(self):
        nb_removed = 0
        origin = Marker(0,0,0,0)

        for frame in self.frames:

            remove_list = []

            for marker in frame.marker_list:
                x_dist = (origin.x - marker.x)
                y_dist = (origin.y - marker.y)
                dist = np.array([x_dist,y_dist, marker.z])

                if np.linalg.norm(dist) < 1:
                    # config.marker_list = config.marker_list.remove(marker)
                    remove_list.append(marker)
                    frame.count -= 1
                    nb_removed += 1
            for r_marker in remove_list:
                frame.marker_list.remove(r_marker)

        print "Filtered ", nb_removed, ' markers'

    def filter_threshold_outside(self, point, threshold):
        nb_removed = 0
        avg = Marker(0, point[0], point[1], point[2])

        print "Trying to filter markers ", threshold, "m outside : ", point
        for frame in self.frames:

            remove_list = []

            for marker in frame.marker_list:
                if math.sqrt(avg.get_dist(marker)) > threshold:
                    remove_list.append(marker)
                    frame.count -= 1
                    nb_removed += 1

            for r_marker in remove_list:
                frame.marker_list.remove(r_marker)

        print "Filtered ", nb_removed, ' markers'

    def filter_negative_x(self):
        nb_removed = 0

        for frame in self.frames:
            remove_list = []

            for marker in frame.marker_list:
                if marker.x < 0.6:
                    remove_list.append(marker)
                    frame.count -= 1
                    nb_removed += 1

            for r_marker in remove_list:
                frame.marker_list.remove(r_marker)

        print "Filtered ", nb_removed, ' markers'

    def track_indices(self):

        # TODO Should read in the configs, figure out # of humans and align objects in pelvis, head, pelvis, head order

        first = None
        # Find first config with num markers * num humans.
        for i, frame in enumerate(self.frames):
            if frame.count == ( NB_HUMAN * NB_MARKERS ):
                print "Found good first frame at : ", i
                self.first_frame = i
                first = self.frames[i]
                break

        if first is None:
            print "Couldn't find a frame with : ", NB_MARKERS, " * ", NB_HUMAN, " markers"
            sys.exit()

        # TODO Check if marker and object timestamps for first_frame match up

        # Get the pevlis frame for each human
        # 
        # 
        #   MOVE THIS INTO LOOP AND CHECK FOR OCCLUSION 
        # 
        # 
        # 

        pelv_frames = []
        for object in first.object_list:
            if 'Pelvis' in object.id:
                pelv_frames.append(object.get_rot_matrix())

        if len(pelv_frames) is not NB_HUMAN:
            print "Did not find the right number of pelvis objects"
            sys.exit()

        # Get the marker map for each human 
        print "Getting marker name map"
        maps = []

        # Need a more intelligent way to find first 18 markers.  This is a total hack
        for pelvis in pelv_frames:
            # Build list of marker numpy arrays
            points = first.get_n_closest_markers(pelvis, NB_MARKERS)
            # for point in points:
            #     for a_point in available:
            #         if a_point.id == point[0]:
            #             available.remove(a_point)

            maps.append(AssignNames(points, pelvis).assign_marker_names())

        # Reorder markers according to map
        new_marker_list = []
        for map in maps:
            print map
            for id in map:
                new_marker_list.append(first.marker_list[id])

        print "Concatenated marker list"
        first.marker_list = new_marker_list
        first.print_marker_ids()


        print "checking duplicates"
        dupes = first.get_duplicate_ids()
        if dupes:
            for dupe in dupes:
                print dupe
            sys.exit()

        # Now all markers are in the proper order, but names aren't set and ids are out of order
        print "Assign marker names"
        first.set_marker_names()

        # Put all ids in ascending order
        for i, marker in enumerate(first.marker_list):
            marker.id = i

        prev = first
        for i in range(self.first_frame+1, self.last_frame):
            curr = self.frames[i]
            new = curr.get_new_config_by_distance(prev)
            self.frames[i] = new
            prev = new



        # for curr in self.frames[self.first_frame+1:self.last_frame]:
        #     new = curr.get_new_config_by_distance(prev)
        #     curr = new
        #     prev = curr

        # # Start fixing markers
        # for i in range( self.first_frame+1, self.last_frame ):
        #     # print "i : " + str(i) + " i-1: " + str(i-1)
        #     prev = self.frames[i-1]
        #     current = self.frames[i]

        #     new = current.get_new_config_by_distance( self.frames, i-1 ) #new config could have duplicates
        #     # new.order_markers()
        #     self.frames[i] = new

    def reorder_ids(self):
        for frame in self.frames:
            new_id = 0
            nb_markers = frame.count
            for marker in frame.marker_list:
                marker.id = new_id
                new_id += 1

    def smooth_markers(self, size):

        if size%2 == 0:
            print "Window size can't be even"
            return

        window = self.frames[:size]

        for i in range(int(floor(size/2)), len(self.frames)-1):
            for m, marker in enumerate(self.frames[i].marker_list):
                avg = np.array([0.0,0.0,0.0])

                for a_frame in window:
                    avg += a_frame.marker_list[m].array

                avg = avg/float(size)

                marker.array = avg
                marker.x = avg[0]
                marker.y = avg[1]
                marker.z = avg[2]

            window = window[1:] + [self.frames[i+1]]

if __name__ == '__main__':
    f = Fixer('/home/rafi/logging_data/third/markers.csv', '/home/rafi/logging_data/third/objects_fixed.csv')

    try:
        with Timer() as t:
            f.load_file()

            # avg = f.get_average_position()
            # f.filter_threshold_outside(avg, 1.8)

            f.filter_negative_x()

            f.filter_pillar()
            print "starting fixing ids"
            f.reorder_ids()

            print "starting to track ids"
            f.track_indices()

            # print "Trying to smooth markers"
            # f.smooth_markers(7)

            nb_diff = 0.0
            for i,frame in enumerate(f.frames):
                if len(frame.marker_list) != NB_MARKERS:
                    # print i, ' ', len(frame.marker_list)
                    nb_diff += 1
            print "% different : " , nb_diff/f.last_frame

            f.save_file()
    finally:
        print('Marker matching took %.03f sec.' % t.interval)