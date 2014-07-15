import numpy as np
from TransformMatrix import *

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

class AssignNames:
    def __init__(self, point_list, pelvis_frame):
        self.count = len(point_list)
        self.points = [None]*self.count
        self.pelv_frame = pelvis_frame

        for i, p in enumerate(point_list):
            self.points[i] = Point( i, p )

    def assign_marker_names(self):
        self.put_points_in_frame(self.pelv_frame)
        body = copy(self.points)

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
        self.points[rPalm.original_id].new_id = 10
        # print 'rPalm : ', rPalm.original_id

        # Wrist
        rWrists = r_arm_points[1:3]
        rWristOuter = max(rWrists, key=lambda p: p.x)
        # print 'rWristOuter : ', rWristOuter.original_id
        self.points[rWristOuter.original_id].new_id = 8

        rWristInner = min(rWrists, key=lambda p: p.x)
        self.points[rWristInner.original_id].new_id = 9
        # print 'rWristInner : ', rWristInner.original_id

        # Elbows
        Elbows = r_arm_points[3:5]
        rElbowOuter = min(Elbows, key=lambda p: p.y)
        # print 'rElbowOuter : ', rElbowOuter.original_id
        self.points[rElbowOuter.original_id].new_id = 6

        rElbowInner = max(Elbows, key=lambda p: p.y)
        # print 'rElbowInner : ', rElbowInner.original_id
        self.points[rElbowInner.original_id].new_id = 7

        # Shoulder
        Shoulders = r_arm_points[5:]
        rShoulderFront = max(Shoulders, key=lambda p: p.x)
        # print 'rShoulderFront : ', rShoulderFront.original_id
        self.points[rShoulderFront.original_id].new_id = 4

        rShoulderBack = min(Shoulders, key=lambda p: p.x)
        # print 'rShoulderBack : ', rShoulderBack.original_id
        self.points[rShoulderBack.original_id].new_id = 5

    def assign_left_arm_names(self, l_arm_points):
        l_arm_points.sort( key=lambda p: p.z, reverse=False )

        # Palm has to be first point in the list
        lPalm = l_arm_points[0]
        self.points[lPalm.original_id].new_id = 17
        # print 'lPalm : ', lPalm.original_id

        # Wrist
        lWrists = l_arm_points[1:3]
        lWristOuter = max(lWrists, key=lambda p: p.x)
        # print 'lWristOuter : ', lWristOuter.original_id
        self.points[lWristOuter.original_id].new_id = 15

        lWristInner = min(lWrists, key=lambda p: p.x)
        self.points[lWristInner.original_id].new_id = 16
        # print 'lWristInner : ', lWristInner.original_id

        # Elbows
        Elbows = l_arm_points[3:5]
        lElbowOuter = max(Elbows, key=lambda p: p.y)
        # print 'lElbowOuter : ', lElbowOuter.original_id
        self.points[lElbowOuter.original_id].new_id = 13

        lElbowInner = min(Elbows, key=lambda p: p.y)
        # print 'lElbowInner : ', lElbowInner.original_id
        self.points[lElbowInner.original_id].new_id = 14

        # Shoulder
        Shoulders = l_arm_points[5:]
        lShoulderFront = max(Shoulders, key=lambda p: p.x)
        # print 'lShoulderFront : ', lShoulderFront.original_id
        self.points[lShoulderFront.original_id].new_id = 11

        lShoulderBack = min(Shoulders, key=lambda p: p.x)
        # print 'lShoulderBack : ', lShoulderBack.original_id
        self.points[lShoulderBack.original_id].new_id = 12

    def assign_body_names(self, body_points):
        body_points.sort( key=lambda p: p.z, reverse=False )

        # Chest
        Chest = body_points[0:2]
        ChestFront = max(Chest, key=lambda p: p.x)
        # print 'ChestFront : ', ChestFront.original_id
        self.points[ChestFront.original_id].new_id = 0

        ChestBack = min(Chest, key=lambda p: p.x)
        self.points[ChestBack.original_id].new_id = 1
        # print 'ChestBack : ', ChestBack.original_id

        # Sternum
        Sternum = body_points[2:]
        SternumFront = max(Sternum, key=lambda p: p.x)
        # print 'SternumFront : ', SternumFront.original_id
        self.points[SternumFront.original_id].new_id = 2

        SternumBack = min(Sternum, key=lambda p: p.x)
        # print 'SternumBack : ', SternumBack.original_id
        self.points[SternumBack.original_id].new_id = 3



if __name__ == '__main__':
    frame1 = [array([ 1.65325708,  1.37396851,  1.20373303]), array([ 1.6390116 ,  1.27223669,  0.93563806]), array([ 1.86381799,  1.16290051,  1.53966638]), array([ 2.16299512,  1.16086987,  1.2172113 ]), array([ 2.01280164,  1.16002686,  1.56070349]), array([ 1.76099866,  1.27423047,  1.55235876]), array([ 1.82515088,  1.38088232,  1.54303247]), array([ 2.06392773,  1.26104333,  1.54999646]), array([ 1.92153613,  1.34364905,  1.50985547]), array([ 2.10540112,  1.04296875,  0.9450249 ]), array([ 1.82304919,  1.08573596,  1.33608301]), array([ 2.13499292,  1.05458667,  0.87096088]), array([ 1.62873596,  1.29743982,  0.86058557]), array([ 1.91677747,  1.35219287,  1.27895422]), array([ 2.07953735,  1.20227856,  1.17632043]), array([ 1.67432178,  1.33002332,  0.91030298]), array([ 2.12015332,  1.11484167,  0.92700696]), array([ 1.74878296,  1.37184265,  1.17296777])]


    pelv_frame = MakeTransform( np.matrix([ [-0.25657, 0.966526, 0], [-0.966526, -0.25657, 0], [0, 0, 1] ]), np.matrix( [1.93278, 1.34812, 1.07163] ) )

    n = AssignNames(frame1, pelv_frame)
    map = n.assign_marker_names()

    print map
















