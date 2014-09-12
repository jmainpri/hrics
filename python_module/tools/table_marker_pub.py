#!/usr/bin/env python

#   Calder Phillips-Grafflin - WPI/ARC Lab

import rospy
import math
import tf
from tf.transformations import *
from visualization_msgs.msg import *

class TableMarkerPublisher:

    def __init__(self, root_frame, rate):
        self.root_frame = root_frame
        self.rate = rate
        self.marker_pub = rospy.Publisher("table_markers", MarkerArray)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.display_table()
            rate.sleep()

    def display_table(self):
        marker_msg = MarkerArray()
        # Make table top
        tabletop = Marker()
        tabletop.type = Marker.CUBE
        tabletop.ns = "table"
        tabletop.id = 1
        tabletop.action = Marker.ADD
        tabletop.lifetime = rospy.Duration(0.2)
        tabletop.header.stamp = rospy.Time.now()
        tabletop.header.frame_id = self.root_frame
        tabletop.scale.x = 1.0
        tabletop.scale.y = 2.2
        tabletop.scale.z = 0.025
        tabletop.color.a = 1.0
        tabletop.color.r = 0.66
        tabletop.color.b = 0.66
        tabletop.color.g = 0.66
        tabletop.pose.position.x = 1.40-0.2
        tabletop.pose.position.y = 0.28
        tabletop.pose.position.z = 0.71
        tabletop.pose.orientation.x = 0.0
        tabletop.pose.orientation.y = 0.0
        tabletop.pose.orientation.z = 0.0
        tabletop.pose.orientation.w = 1.0
        marker_msg.markers.append(tabletop)
        # Make leg 1
        leg1 = Marker()
        leg1.type = Marker.CUBE
        leg1.ns = "table"
        leg1.id = 2
        leg1.action = Marker.ADD
        leg1.lifetime = rospy.Duration(0.2)
        leg1.header.stamp = rospy.Time.now()
        leg1.header.frame_id = self.root_frame
        leg1.scale.x = 0.05
        leg1.scale.y = 0.05
        leg1.scale.z = 0.70
        leg1.color.a = 1.0
        leg1.color.r = 0.66
        leg1.color.b = 0.66
        leg1.color.g = 0.66
        leg1.pose.position.x = 1.80-0.2
        leg1.pose.position.y = 1.28
        leg1.pose.position.z = 0.35
        leg1.pose.orientation.x = 0.0
        leg1.pose.orientation.y = 0.0
        leg1.pose.orientation.z = 0.0
        leg1.pose.orientation.w = 1.0
        marker_msg.markers.append(leg1)
        # Make leg 2
        leg2 = Marker()
        leg2.type = Marker.CUBE
        leg2.ns = "table"
        leg2.id = 3
        leg2.action = Marker.ADD
        leg2.lifetime = rospy.Duration(0.2)
        leg2.header.stamp = rospy.Time.now()
        leg2.header.frame_id = self.root_frame
        leg2.scale.x = 0.05
        leg2.scale.y = 0.05
        leg2.scale.z = 0.70
        leg2.color.a = 1.0
        leg2.color.r = 0.66
        leg2.color.b = 0.66
        leg2.color.g = 0.66
        leg2.pose.position.x = 1.0-0.2
        leg2.pose.position.y = 1.28
        leg2.pose.position.z = 0.35
        leg2.pose.orientation.x = 0.0
        leg2.pose.orientation.y = 0.0
        leg2.pose.orientation.z = 0.0
        leg2.pose.orientation.w = 1.0
        marker_msg.markers.append(leg2)
        # Make leg 3
        leg3 = Marker()
        leg3.type = Marker.CUBE
        leg3.ns = "table"
        leg3.id = 4
        leg3.action = Marker.ADD
        leg3.lifetime = rospy.Duration(0.2)
        leg3.header.stamp = rospy.Time.now()
        leg3.header.frame_id = self.root_frame
        leg3.scale.x = 0.05
        leg3.scale.y = 0.05
        leg3.scale.z = 0.70
        leg3.color.a = 1.0
        leg3.color.r = 0.66
        leg3.color.b = 0.66
        leg3.color.g = 0.66
        leg3.pose.position.x = 1.80-0.2
        leg3.pose.position.y = -0.72
        leg3.pose.position.z = 0.35
        leg3.pose.orientation.x = 0.0
        leg3.pose.orientation.y = 0.0
        leg3.pose.orientation.z = 0.0
        leg3.pose.orientation.w = 1.0
        marker_msg.markers.append(leg3)
        # Make leg 4
        leg4 = Marker()
        leg4.type = Marker.CUBE
        leg4.ns = "table"
        leg4.id = 5
        leg4.action = Marker.ADD
        leg4.lifetime = rospy.Duration(0.2)
        leg4.header.stamp = rospy.Time.now()
        leg4.header.frame_id = self.root_frame
        leg4.scale.x = 0.05
        leg4.scale.y = 0.05
        leg4.scale.z = 0.70
        leg4.color.a = 1.0
        leg4.color.r = 0.66
        leg4.color.b = 0.66
        leg4.color.g = 0.66
        leg4.pose.position.x = 1.0-0.2
        leg4.pose.position.y = -0.72
        leg4.pose.position.z = 0.35
        leg4.pose.orientation.x = 0.0
        leg4.pose.orientation.y = 0.0
        leg4.pose.orientation.z = 0.0
        leg4.pose.orientation.w = 1.0
        marker_msg.markers.append(leg4)
        self.marker_pub.publish(marker_msg)

if __name__ == "__main__":
    rospy.init_node("table_marker_publisher")
    rospy.loginfo("Starting the table marker broadcaster...")
    #Get the parameters from the server
    root_frame = rospy.get_param("~root_frame", "base_footprint")
    rate = rospy.get_param("~rate", 10.0)
    TableMarkerPublisher(root_frame, rate)
