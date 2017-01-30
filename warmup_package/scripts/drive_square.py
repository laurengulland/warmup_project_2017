#!/usr/bin/env python
"""Attempting to drive a 1mx1m square"""

from geometry_msgs.msg import Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry
import rospy
import tf

class DriveSquare(object):
    def __init__(self):
        #initialize node
        rospy.init_node('drivesquare')
        #initiate Subs/Pubs
        rospy.Subscriber('/odom', Odometry, self.processOdom)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #initalize flags
        self.r = rospy.Rate(10) #execute at 10 Hz
        self.moves = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0))
        self.time_elapsed = 0 #in seconds
        self.turns = 0 #how many times has turned

    def processOdom(self,msg):
        print msg

    def drive_motherfucker(self):
        if self.time_elapsed > 3:
            self.turns += 1
            #turn 90deg here????
            self.time_elapsed = 0 #reset time elapsed
        if self.turns > 4: #stop fully after turning four times
            self.moves.linear.x = 0.0
            self.moves.angular.z = 0.0

    def fuckingstop(self):
        self.moves.linear.x = 0.0
        self.moves.angular.z = 0.0
        self.pub.publish(self.moves)

    def run(self):
        rospy.on_shutdown(self.fuckingstop)
        while not rospy.is_shutdown():
            # set some kind of local time
            #set time elapsed from 0 in seconds to update
            self.drive_motherfucker()
            self.pub.publish(self.moves)
            self.r.sleep()
        print "Node is finished"

squaaaaare=DriveSquare()
squaaaaare.run()
