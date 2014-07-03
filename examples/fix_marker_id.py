#!/usr/bin/python

import numpy as np
import os
import collections
import operator
import time

THRESHOLD = 0.0025

class Timer:
    def __enter__(self):
        self.start = time.clock()
        return self

    def __exit__(self, *args):
        self.end = time.clock()
        self.interval = self.end - self.start

class Marker:
    def __init__(self, id, x, y, z):
        self.id = id
        self.x = x
        self.y = y
        self.z = z

    def are_similar(self, a_marker, threshold):
        if np.linalg.norm( self.numpy() - a_marker.numpy() ) > threshold:
            return False
        else:
            return True

    def numpy(self):
        return np.array( [self.x,self.y,self.z] )

    def get_dist(self, other):
        return np.linalg.norm(self.numpy() - other.numpy())

class Frame:
    def __init__(self, t_sec, t_usec, nb_markers, marker_list):
        self.sec = t_sec
        self.usec = t_usec
        self.count = nb_markers
        self.marker_list = marker_list


    def order_markers(self):
        new_list = [None]*len(self.marker_list)
        for marker in self.marker_list:
            new_list[marker.id] = marker

        self.marker_list = new_list

    def print_marker_ids(self):
        marker_string = ''

        for marker in self.marker_list:
            marker_string += str(marker.id) + ' '
        print marker_string

    def same_indices(self, other):
        if (self.count != other.count):
            return False

        for i in range(self.count):

            if self.marker_list[i] == None:
                return False

            if not self.marker_list[i].are_similar(other.marker_list[i], THRESHOLD):
                return False

        return True

    def find_closest(self, other):
        closest_dist = float("inf")
        closest_marker = Marker( 0, float("inf"), float("inf"), float("inf")  )

        for marker in self.marker_list:
            dist = marker.get_dist(other)
            #print "dist : " + str(dist) + "marker 1 : " + str(marker.id) + " marker 2 " + str(other.id)

            if dist < closest_dist:
                closest_dist = dist
                closest_marker = marker

        return closest_marker

    # Returns a list of tuples sorted by lowest value (distance)
    # [ ( key, val), ... etc
    def get_distances(self, other):
        dist = {}
        for marker in self.marker_list:
            dist[marker.id] = marker.get_dist(other)

        return sorted(dist.items(), key = lambda (k,v): v)

    def get_marker_by_id(self, id):
        for marker in self.marker_list:
            if marker.id == id:
                return marker

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


    def has_empty_marker(self):
        nb_empty = 0

        for i, marker in enumerate(self.marker_list):
            if marker == 0:
                nb_empty += 1
        return nb_empty

    def get_duplicate_ids(self):
        indices = []

        for marker in self.marker_list:
            if not marker == None:
                indices.append(marker.id)

        return [x for x, y in collections.Counter(indices).items() if y > 1]

    def get_new_config_by_distance(self, prev):
        dist_list = [None]*len(self.marker_list)
        for i, marker in enumerate(self.marker_list):
            dist_list[i] = prev.get_distances(marker)

        new_markers = [None]*len(prev.marker_list)
        shortest_found = [ float('inf') ]*len(prev.marker_list)


        for i, marker in enumerate(self.marker_list):
            # Tuple of (id, dist)
            closest_id = dist_list[i][0][0]
            closest_dist = dist_list[i][0][1]

            if closest_dist < 0.5  and closest_id < len(prev.marker_list) and shortest_found[closest_id] > closest_dist:
            # if shortest_found[closest_id] > closest_dist:
            # if closest_dist < 0.05  and closest_id < len(prev.marker_list):
                new_markers[closest_id] =  Marker( closest_id, marker.x, marker.y, marker.z )
                shortest_found[closest_id] = closest_dist

        for i in range(len(prev.marker_list)):
            if new_markers[i] == None:
                new_markers[i] = prev.marker_list[i]



        temp = Frame( self.sec, self.usec, prev.count, new_markers )

        return temp

    def fill_empty_markers(self, prev_config):
        for i in range(0, self.count):
            if self.marker_list[i] == 0:
                self.marker_list[i] = prev_config.get_marker_by_id(i)



class Fixer:

    def __init__(self, filepath):
        self.filepath = filepath
        self.configs = []
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

                self.configs.append( Frame(sec, nsec, count, markers) )
            self.last_config = len(self.configs)


        print "# configs loaded : " + str(len(self.configs))

    def save_file(self):
        print "Trying to output new file"

        #  Get the out filename
        dir, path = os.path.split(self.filepath)
        name, type = path.rsplit('.', 1)
        outpath = name + '_fixed.'+type

        with open(outpath, 'w') as f:
            for config in self.configs:
                line_str = ""
                line_str += str(config.sec) + ','
                line_str += str(config.usec) + ','
                line_str += str(config.count) + ','

                for marker in config.marker_list:
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

        for config in self.configs:
            for marker in config.marker_list:
                m_tot += marker.numpy()
                m_count += 1

        print "trying to finding avg pos"

        return m_tot/m_count

    def filter_ghosts(self, point, threshold):
        count = 0
        avg = Marker(0, point[0], point[1], point[2])

        print "Trying to filter markers ", threshold, "m from : ", point
        for config in self.configs:
            for marker in config.marker_list:
                if avg.get_dist(marker) > threshold:
                    config.marker_list.remove(marker)
                    count += 1

        print "Filtered ", count, ' markers'



    def match_indices(self):
        # Find the first agreeing Frames
        last = self.configs[0]
        for i in range( 1, self.last_config) :
            if last.same_indices( self.configs[i] ):
                self.max_markers = last.count
                self.first_config = i-1
                print "Frames " + str(i-1) +' and ' + str(i) + ' have the same indices'
                break

            last = self.configs[i]

        #self.last_config = 13

        #Match the indices
        for i in range( self.first_config+1, self.last_config ):
            # print "i : " + str(i) + " i-1: " + str(i-1)
            prev = self.configs[i-1]
            current = self.configs[i]

            #  Nothing to fix
            if prev.same_indices(current):
                # print "nothing to fix"
                continue

            # fixed = Frame( current.sec, current.usec, self.max_markers, [0]*self.max_markers )

            new = current.get_new_config_by_distance( prev ) #new config could have duplicates
            new.order_markers()
            self.configs[i] = new


if __name__ == '__main__':
    f = Fixer('/home/rafi/Desktop/positions.csv')

try:
    with Timer() as t:
        f.load_file()
        avg = f.get_average_position()
        f.filter_ghosts(avg, 2.5)
        f.match_indices()
        f.save_file()
finally:
    print('Marker matching took %.03f sec.' % t.interval)