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


THRESHOLD = 0.0025
# Experiment is 18 per human
N_MARKERS = 36

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

        return None

    def print_marker_ids(self):
        marker_string = ''

        for marker in self.marker_list:
            marker_string += str(marker.id) + ' '
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

    def get_new_config_by_distance(self, configs, prev_index):
        # print "i : ", prev_index+1
        prev = configs[prev_index]

        new_markers = [None]*prev.count
        shortest_found = [float('inf')]*prev.count

        for i, marker in enumerate(self.marker_list):

            dists = prev.get_distances(marker)

            for closest in dists:
                closest_id = closest[0]
                closest_dist = closest[1]

                if closest_id < prev.count and shortest_found[closest_id] > closest_dist:
                    new_markers[closest_id] = Marker(closest_id, marker.x, marker.y, marker.z, prev.marker_list[closest_id].name)
                    shortest_found[closest_id] = closest_dist
                    break

        for i in range(0, prev.count):
            if new_markers[i] is None:  # Marker dropped!
                new_markers[i] = prev.get_marker_by_id(i)

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

    def get_n_closest_markers(self, pelv_frame, n):

        points = []
        pelv_point = np.transpose(pelv_frame[:,3])
        pelv_marker = Marker( 0, pelv_point[0,0], pelv_point[0,1], pelv_point[0,2]  )

        closest = self.get_distances(pelv_marker)[:n]
        # Put the markers back in order by id
        closest.sort( key = lambda p: p[0], reverse=False )

        for pair in closest:
            points.append( self.marker_list[pair[0]].array )

        return points


class Fixer:

    def __init__(self, m_filepath, o_filepath):
        self.m_filepath = m_filepath
        self.o_filepath = o_filepath
        self.frames = []
        self.max_markers = None
        self.first_config = 0
        self.last_config = 0

    def load_file(self):
        print "Trying to open file"

        marker_file = []
        object_file = []

        with open(self.m_filepath, 'r') as m_file:
            with open(self.o_filepath, 'r') as o_file:

                marker_file = [row for row in csv.reader(m_file, delimiter=',')]
                object_file = [row for row in csv.reader(o_file, delimiter=',')]

        nb_lines = min(len(marker_file), len(object_file))
        self.last_config = nb_lines

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

        # with open(self.m_filepath, 'r') as m_file:
        #     with open(self.o_filepath, 'r') as o_file:
        #         marker_rows = m_file.readlines()
        #         object_rows = o_file.readlines()
        #
        #
        #         for line in m_file:
        #             markers = []
        #
        #             cell = line.split(',')
        #             sec = float(cell[0])
        #             nsec = float(cell[1])
        #             count = int(cell[2])
        #
        #             for i in range(3, count*4, 4):
        #                 id = int(cell[i])
        #                 x = float(cell[i+1])
        #                 y = float(cell[i+2])
        #                 z = float(cell[i+3])
        #                 marker = Marker(id, x, y, z)
        #                 markers.append(marker)
        #
        #         for line in o_file:
        #             objects = []
        #
        #             cell = line.split(',')
        #             sec = float(cell[0])
        #             nsec = float(cell[1])
        #             count = int(cell[2])
        #
        #             for i in range(3, count*9, 9):
        #                 name = str(cell[i])
        #                 occluded = int(cell[i+1])
        #                 x = float(cell[i+2])
        #                 y = float(cell[i+3])
        #                 z = float(cell[i+4])
        #                 r_x = float(cell[i+5])
        #                 r_y = float(cell[i+6])
        #                 r_z = float(cell[i+7])
        #                 r_w = float(cell[i+8])
        #
        #                 object = Object( name, occluded, x, y, z, r_x, r_y, r_z, r_w )
        #                 objects.append(object)
        #
        #
        #
        #             self.frames.append( Frame(sec, nsec, count, markers, objects) )
        #         self.last_config = len(self.frames)

        print "# configs loaded : " + str(len(self.frames))

    def save_file(self):
        print "Trying to output new file"

        #  Get the out filename
        dir, path = os.path.split(self.m_filepath)
        name, type = path.rsplit('.', 1)
        outpath = name + '_fixed.'+type


        # TODO Should output a truncated objects file starting at first_config
        with open(outpath, 'w') as f:
            for frame in self.frames[self.first_config:self.last_config]:
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

    def track_indices(self):
        # Find the first agreeing Frames

        found_agreeing_frames = False

        last = self.frames[0]
        for i in range( 1, self.last_config) :
            # print "len 1 : ", len(last.marker_list), "len 2 : ", len(self.frames[i].marker_list)
            if last.is_similar_frame( self.frames[i] ):
                print "Frames " + str(i-1) +' and ' + str(i) + ' have the same indices'

                self.max_markers = last.count
                self.first_config = i-1
                found_agreeing_frames = True

                # Get pelv frame
                pelv_frames = []
                for object in last.object_list:
                    if object.id == 'TouchTomorrow3':
                        pelv_frames.append(object.get_rot_matrix())

                if pelv_frames is None:
                    print "Couldn't find a pelvis frame!"
                    sys.exit()

                # Get marker name map
                # Check if there are more than 1 humans
                # If so isolate their markers and pass to the namer
                # Get two maps
                print "Getting marker name map"
                maps = []
                for pelvis in pelv_frames:

                    # Build list of marker numpy arrays
                    points = last.get_n_closest_markers(pelvis, 18)

                    namer = AssignNames(points, pelvis)
                    maps.append(namer.assign_marker_names())


                # Reorder markers according to map
                # Make into a function
                # Should take in either one or two map lists

                for map in maps:
                    print map
                    new_marker_list = []
                    for i, id in enumerate(map):
                        new_marker_list.append(last.marker_list[id])
                        new_marker_list[i].id = i

                last.marker_list = new_marker_list

                # print "Fixing marker ids according to map"
                # new_marker_list = []
                # for i, id in enumerate(map):
                #     new_marker_list.append(last.marker_list[id])
                #     new_marker_list[i].id = i
                #
                # last.marker_list = new_marker_list

                # Set marker names by id
                print "Setting marker names"
                last.set_marker_names_by_id()

                break

            last = self.frames[i]

        if not found_agreeing_frames:
            print "Couldn't find two frames with the same # of similar markers.  Try a different N_MARKERS?"
            return

        #Match the indices
        for i in range( self.first_config+1, self.last_config ):
            # print "i : " + str(i) + " i-1: " + str(i-1)
            prev = self.frames[i-1]
            current = self.frames[i]

            #  Nothing to fix
            if prev.is_similar_frame(current):
                continue

            new = current.get_new_config_by_distance( self.frames, i-1 ) #new config could have duplicates
            new.order_markers()
            self.frames[i] = new

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


    # def moving_average(self):
    #
    #     for f, frame in enumerate(self.frames):
    #
    #         new_markers = []
    #
    #         for m in range(len(frame.marker_list)):
    #             marker = frame.marker_list[m]
    #             avg = np.array([0,0,0])
    #
    #             for i in range(-2,3):
    #                 index = f + i
    #                 if index < 0:
    #                     index = 0
    #                 if index >= self.last_config:
    #                     index = self.last_config-1
    #
    #
    #                 # print "nb frames : ", len(self.frames), ' index : ', index, " marker : ", m, " nb markers : ", len(frame.marker_list)
    #                 # print "m : ", m, " len marker list : ", len(self.frames[index].marker_list)
    #
    #                 avg += self.frames[index].marker_list[m].array
    #             avg = avg/5
    #             marker.array = avg
    #             marker.x = avg[0]
    #             marker.y = avg[1]
    #             marker.z = avg[2]

if __name__ == '__main__':
    f = Fixer('/home/rafi/logging_data/second/markers.csv', '/home/rafi/logging_data/second/objects.csv')

    try:
        with Timer() as t:
            f.load_file()

            avg = f.get_average_position()
            f.filter_threshold_outside(avg, 2.0)

            f.filter_pillar()
            print "starting fixing ids"
            f.reorder_ids()

            current_time = time.clock()
            elapsed = current_time - t.start

            print "Trying to match indices.  Current time : ", elapsed

            f.track_indices()

            current_time = time.clock()
            elapsed = current_time - t.start

            print "Finished matching indices.  Current time : ", elapsed

            # print "Trying to smooth with a moving average"
            # current_time = time.clock()
            # elapsed = current_time - t.start
            # f.moving_average()

            print "Trying to smooth markers"
            # f.smooth_markers(7)

            nb_diff = 0.0
            for i,frame in enumerate(f.frames):
                if len(frame.marker_list) != N_MARKERS:
                    # print i, ' ', len(frame.marker_list)
                    nb_diff += 1
            print "% different : " , nb_diff/f.last_config

            f.save_file()
    finally:
        print('Marker matching took %.03f sec.' % t.interval)