#!/usr/bin/env/ python

"""Basics of creating messages in ROS node"""

from visualization_msgs.msg import Marker
import rospy



class Sphero(object):
    def __init__(self):
        rospy.init_node('marker')
        rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.r = rospy.Rate(10)
        self.spherey = Marker()
        self.spherey.header.frame_id = "/Cmap"
        self.spherey.header.stamp    = rospy.get_rostime()
        self.spherey.ns = "robot"
        self.spherey.id = 0
        self.spherey.type = 2 # sphere
        self.spherey.action = 0
        self.spherey.pose.position = self.state.point
        self.spherey.pose.orientation.x = 1
        self.spherey.pose.orientation.y = 2
        self.spherey.pose.orientation.z = 0
        self.spherey.pose.orientation.w = 1.0
        self.spherey.scale.x = 1.0
        self.spherey.scale.y = 1.0
        self.spherey.scale.z = 1.0

        self.spherey.color.r = 0.0
        self.spherey.color.g = 1.0
        self.spherey.color.b = 0.0
        self.spherey.color.a = 1.0
    def run(self):
        while not rospy.is_shutdown():

            self.r.sleep()
        print "Node is finished!"
