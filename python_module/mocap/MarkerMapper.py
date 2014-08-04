import numpy as np
from TransformMatrix import *

#       DEFAULT SETUP
# Markers 
# ChestFront        0
# ChestBack         1
# SternumFront      2
# SternumBack       3
# rShoulderFront    4
# rShoulderBack     5
# rElbowOuter       6
# rElbowInner       7
# rWristOuter       8
# rWristInner       9
# rPalm             10
# lShoulderFront    11
# lShoulderBack     12
# lElbowOuter       13
# lElbowInner       14
# lWristOuter       15
# lWristInner       16
# lPalm             17

# Objects
# Pelvis
# Head

#       ELBOWPAD SETUP
# Markers 
# ChestFront        0
# ChestBack         1
# SternumFront      2
# SternumBack       3
# rShoulderFront    4
# rShoulderBack     5
# rWristOuter       6
# rWristInner       7
# rPalm             8
# lShoulderFront    9
# lShoulderBack     10
# lWristOuter       11
# lWristInner       12
# lPalm             13

# Objects
# Pelvis
# Head
# rElbow
# lElbow

class Point:
    def __init__(self, id, point):
        self.original_id = id
        self.new_id = None
        self.x = point[0]
        self.y = point[1]
        self.z = point[2]
        self.array = point

    def get_dist(self, other):
        diff = self.array - other.array
        return math.sqrt( np.dot(diff, diff.conj()) )

class MarkerMapper:
    def __init__(self, point_list, pelvis_frame):
        self.count = len(point_list)
        self.points = [None]*self.count
        self.pelv_frame = pelvis_frame

        for i, p in enumerate(point_list):
            self.points[i] = Point( p[0], p[1] )

    def assign_marker_names(self, elbow_pads, r_arm_only):
        self.put_points_in_frame(self.pelv_frame)
        body = copy(self.points)

        # Using an elbow pad object instead of default marker setup
        if elbow_pads:
            if r_arm_only:
                # Get the right arm : first 5 points with negative y
                r_arm = sorted(body, key=lambda p: p.y, reverse=False)[0:5]
                # Remove the right arm from the body list
                body = [ point for point in body if point not in r_arm ]

                # Assign names from each list
                self.assign_right_arm_names_pads(r_arm)
                self.assign_body_names(body)
            else:
                print "Haven't set this up yet"
                # Get the right arm : first 5 points with negative y
                r_arm = sorted(body, key=lambda p: p.y, reverse=False)[0:5]
                # Remove the right arm from the body list
                body = [ point for point in body if point not in r_arm ]

                # Get the left arm
                l_arm = sorted(body, key=lambda p: p.y, reverse=True)[0:5]
                # Remove the left arm from the body list
                body = [ point for point in body if point not in l_arm ]

                # Assign names from each list
                self.assign_right_arm_names_pads(r_arm)
                self.assign_left_arm_names_pads(l_arm)
                self.assign_body_names(body)
        # Using default marker setup
        else:
            if r_arm_only:
                # Get the right arm : first 7 points with negative y
                r_arm = sorted(body, key=lambda p: p.y, reverse=False)[0:7]
                # Remove the right arm from the body list
                body = [ point for point in body if point not in r_arm ]

                # Assign names from each list
                self.assign_right_arm_names(r_arm)
                self.assign_body_names(body)
            else:
                # Get the right arm : first 7 points with negative y
                r_arm = sorted(body, key=lambda p: p.y, reverse=False)[0:7]
                # Remove the right arm from the body list
                body = [ point for point in body if point not in r_arm ]

                # Get the left arm
                l_arm = sorted(body, key=lambda p: p.y, reverse=True)[0:7]
                # Remove the left arm from the body list
                body = [ point for point in body if point not in l_arm ]

                # Assign names from each list
                self.assign_right_arm_names(r_arm)
                self.assign_left_arm_names(l_arm)
                self.assign_body_names(body)

        id_map = [None]*self.count
        for point in self.points:
            id_map[point.new_id] = point.original_id

        return id_map

    # Modifies the point list in place.
    def put_points_in_frame(self, frame):
        inv = np.linalg.inv(frame)

        for point in self.points:
            new_point = np.array(np.array(inv).dot(np.array(append(point.array, 1.0))))[0:3]
            point.array = new_point
            point.x = new_point[0]
            point.y = new_point[1]
            point.z = new_point[2]

    def assign_right_arm_names(self, r_arm_points):
        r_arm_points.sort( key=lambda p: p.z, reverse=False )

        # Palm has to be first point in the list
        rPalm = r_arm_points[0]
        self.get_point_by_id(rPalm.original_id).new_id = 10
        # print 'rPalm : ', rPalm.original_id

        # Wrist
        rWrists = r_arm_points[1:3]
        rWristOuter = max(rWrists, key=lambda p: p.x)
        # print 'rWristOuter : ', rWristOuter.original_id
        self.get_point_by_id(rWristOuter.original_id).new_id = 8

        rWristInner = min(rWrists, key=lambda p: p.x)
        self.get_point_by_id(rWristInner.original_id).new_id = 9
        # print 'rWristInner : ', rWristInner.original_id

        # Elbows
        Elbows = r_arm_points[3:5]
        rElbowOuter = min(Elbows, key=lambda p: p.y)
        # print 'rElbowOuter : ', rElbowOuter.original_id
        self.get_point_by_id(rElbowOuter.original_id).new_id = 6

        rElbowInner = max(Elbows, key=lambda p: p.y)
        # print 'rElbowInner : ', rElbowInner.original_id
        self.get_point_by_id(rElbowInner.original_id).new_id = 7

        # Shoulder
        Shoulders = r_arm_points[5:]
        rShoulderFront = max(Shoulders, key=lambda p: p.x)
        # print 'rShoulderFront : ', rShoulderFront.original_id
        self.get_point_by_id(rShoulderFront.original_id).new_id = 4

        rShoulderBack = min(Shoulders, key=lambda p: p.x)
        # print 'rShoulderBack : ', rShoulderBack.original_id
        self.get_point_by_id(rShoulderBack.original_id).new_id = 5

    def assign_right_arm_names_pads(self, r_arm_points):
        r_arm_points.sort( key=lambda p: p.z, reverse=False )

        # Palm has to be first point in the list
        rPalm = r_arm_points[0]
        self.get_point_by_id(rPalm.original_id).new_id = 8
        # print 'rPalm : ', rPalm.original_id

        # Wrist
        rWrists = r_arm_points[1:3]
        rWristOuter = max(rWrists, key=lambda p: p.x)
        # print 'rWristOuter : ', rWristOuter.original_id
        self.get_point_by_id(rWristOuter.original_id).new_id = 6

        rWristInner = min(rWrists, key=lambda p: p.x)
        self.get_point_by_id(rWristInner.original_id).new_id = 7
        # print 'rWristInner : ', rWristInner.original_id

        # Shoulder
        Shoulders = r_arm_points[3:]
        rShoulderFront = max(Shoulders, key=lambda p: p.x)
        # print 'rShoulderFront : ', rShoulderFront.original_id
        self.get_point_by_id(rShoulderFront.original_id).new_id = 4

        rShoulderBack = min(Shoulders, key=lambda p: p.x)
        # print 'rShoulderBack : ', rShoulderBack.original_id
        self.get_point_by_id(rShoulderBack.original_id).new_id = 5

    def assign_left_arm_names(self, l_arm_points):
        l_arm_points.sort( key=lambda p: p.z, reverse=False )

        # Palm has to be first point in the list
        lPalm = l_arm_points[0]
        self.get_point_by_id(lPalm.original_id).new_id = 17
        # print 'lPalm : ', lPalm.original_id

        # Wrist
        lWrists = l_arm_points[1:3]
        lWristOuter = max(lWrists, key=lambda p: p.x)
        # print 'lWristOuter : ', lWristOuter.original_id
        self.get_point_by_id(lWristOuter.original_id).new_id = 15

        lWristInner = min(lWrists, key=lambda p: p.x)
        self.get_point_by_id(lWristInner.original_id).new_id = 16
        # print 'lWristInner : ', lWristInner.original_id

        # Elbows
        Elbows = l_arm_points[3:5]
        lElbowOuter = max(Elbows, key=lambda p: p.y)
        # print 'lElbowOuter : ', lElbowOuter.original_id
        self.get_point_by_id(lElbowOuter.original_id).new_id = 13

        lElbowInner = min(Elbows, key=lambda p: p.y)
        # print 'lElbowInner : ', lElbowInner.original_id
        self.get_point_by_id(lElbowInner.original_id).new_id = 14

        # Shoulder
        Shoulders = l_arm_points[5:]
        lShoulderFront = max(Shoulders, key=lambda p: p.x)
        # print 'lShoulderFront : ', lShoulderFront.original_id
        self.get_point_by_id(lShoulderFront.original_id).new_id = 11

        lShoulderBack = min(Shoulders, key=lambda p: p.x)
        # print 'lShoulderBack : ', lShoulderBack.original_id
        self.get_point_by_id(lShoulderBack.original_id).new_id = 12

    def assign_left_arm_names_pads(self, l_arm_points):
        l_arm_points.sort( key=lambda p: p.z, reverse=False )

        # Palm has to be first point in the list
        lPalm = l_arm_points[0]
        self.get_point_by_id(lPalm.original_id).new_id = 13
        # print 'lPalm : ', lPalm.original_id

        # Wrist
        lWrists = l_arm_points[1:3]
        lWristOuter = max(lWrists, key=lambda p: p.x)
        # print 'lWristOuter : ', lWristOuter.original_id
        self.get_point_by_id(lWristOuter.original_id).new_id = 11

        lWristInner = min(lWrists, key=lambda p: p.x)
        self.get_point_by_id(lWristInner.original_id).new_id = 12
        # print 'lWristInner : ', lWristInner.original_id

        # Shoulder
        Shoulders = l_arm_points[3:]
        lShoulderFront = max(Shoulders, key=lambda p: p.x)
        # print 'lShoulderFront : ', lShoulderFront.original_id
        self.get_point_by_id(lShoulderFront.original_id).new_id = 9

        lShoulderBack = min(Shoulders, key=lambda p: p.x)
        # print 'lShoulderBack : ', lShoulderBack.original_id
        self.get_point_by_id(lShoulderBack.original_id).new_id = 10

    def assign_body_names(self, body_points):
        body_points.sort( key=lambda p: p.z, reverse=False )

        # Chest
        Chest = body_points[0:2]
        ChestFront = max(Chest, key=lambda p: p.x)
        # print 'ChestFront : ', ChestFront.original_id
        self.get_point_by_id(ChestFront.original_id).new_id = 0

        ChestBack = min(Chest, key=lambda p: p.x)
        self.get_point_by_id(ChestBack.original_id).new_id = 1
        # print 'ChestBack : ', ChestBack.original_id

        # Sternum
        Sternum = body_points[2:]
        SternumFront = max(Sternum, key=lambda p: p.x)
        # print 'SternumFront : ', SternumFront.original_id
        self.get_point_by_id(SternumFront.original_id).new_id = 2

        SternumBack = min(Sternum, key=lambda p: p.x)
        # print 'SternumBack : ', SternumBack.original_id
        self.get_point_by_id(SternumBack.original_id).new_id = 3

    def get_point_by_id(self, id):
        for point in self.points:
            if point.original_id == id:
                return point
