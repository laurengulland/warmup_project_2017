#!/usr/bin/env python
"""Attempting to drive a 1mx1m square"""

from geometry_msgs.msg import Twist, Vector3
import rospy

class DriveSquare(object):
    def __init__(self):
        #initialize node
        rospy.init_node('drivesquare')
        #initiate Subs/Pubs
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #initalize flags
        self.settings = termios.tcgetattr(sys.stdin)
        self.r = rospy.Rate(10) #execute at 10 Hz
        self.linear_vel = None
        self.angular_vel = None
        self.time_elapsed = 0 #in seconds
        self.turns = 0 #how many times has turned

    def drive_motherfucker(self):
        if self.time_elapsed > 3:
            self.turns += 1
            #turn 90deg here????
            self.time_elapsed = 0 #reset time elapsed
        if self.turns > 4: #stop fully after turning four times
            self.linear_vel = Vector3(x = 0.0)
            self.angular_vel = Vector3(z = 0.0)


    def run(self):
        while not rospy.is_shutdown():
            # set some kind of local time
            #set time elapsed from 0 in seconds to update

            self.drive_motherfucker()
            twist_msg = Twist(linear=self.linear_vel, angular = self.angular_vel)
            print twist_msg
            self.pub.publish(twist_msg)
            self.r.sleep()
        print "Node is finished"

squaaaaare=drivesquare()
squaaaaare.run()
