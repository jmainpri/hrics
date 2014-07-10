#!/usr/bin/python


from TransformMatrix import *
import numpy as np
import os
import collections
import time
import math

THRESHOLD = 0.0025
# Experiment is 18 per human
N_MARKERS = 18

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


class Frame:
    def __init__(self, t_sec, t_usec, nb_markers, marker_list):
        self.sec = t_sec
        self.usec = t_usec
        self.count = nb_markers
        self.marker_list = marker_list

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
        temp = Frame(self.sec, self.usec, len(new_markers), new_markers)

        return temp

    def assign_marker_names(self, pelv_frame):
        pt_list = []

        pelv_frame = MakeTransform( np.matrix([ [-0.25657, 0.966526, 0], [-0.966526, -0.25657, 0], [0, 0, 1] ]), np.matrix( [1.93278, 1.34812, 1.07163] ) )
        pelv_inv = np.linalg.inv(pelv_frame)


        # Get a list of numpy arrays instead of markers
        for marker in self.marker_list:
            pt_list.append(marker.array)

        # Put all points in the pelvis frame
        for point in pt_list:
            point = point * pelv_frame






        # Re-arrange IDs by name.  This needs to be done in both matching configs

    # def get_markers_in_torso_frame(self):
    


class Fixer:

    def __init__(self, filepath):
        self.filepath = filepath
        self.frames = []
        self.max_markers = None
        self.first_config = 0
        self.last_config = 0

    def load_file(self):
        print "Trying to open file"

        with open(self.filepath, 'r') as f:
            for line in f:
                markers = []

                cell = line.split(',')
                sec = float(cell[0])
                nsec = float(cell[1])
                count = int(cell[2])

                for i in range(3, count*4, 4):
                    id = int(cell[i])
                    x = float(cell[i+1])
                    y = float(cell[i+2])
                    z = float(cell[i+3])
                    marker = Marker(id, x, y, z)
                    markers.append(marker)

                self.frames.append( Frame(sec, nsec, count, markers) )
            self.last_config = len(self.frames)

        print "# configs loaded : " + str(len(self.frames))

    def save_file(self):
        print "Trying to output new file"

        #  Get the out filename
        dir, path = os.path.split(self.filepath)
        name, type = path.rsplit('.', 1)
        outpath = name + '_fixed.'+type

        with open(outpath, 'w') as f:
            for frame in self.frames:
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

                self.frames[self.first_config].assign_marker_names('poop')
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

if __name__ == '__main__':
    f = Fixer('/home/rafi/logging_data/second/markers.csv')

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

            nb_diff = 0.0
            for i,frame in enumerate(f.frames):
                if len(frame.marker_list) != N_MARKERS:
                    # print i, ' ', len(frame.marker_list)
                    nb_diff += 1
            print "% different : " , nb_diff/f.last_config

            f.save_file()
    finally:
        print('Marker matching took %.03f sec.' % t.interval)