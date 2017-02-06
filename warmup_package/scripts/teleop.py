#!/usr/bin/env python
"""Working Teleop Code"""

from geometry_msgs.msg import Twist, Vector3
import rospy
import tty #for key gathering
import select #for key gathering
import sys #for key gathering
import termios #for key gathering

class Teleop(object):
    def __init__(self):
        #initialize node
        rospy.init_node('teleop')
        #initiate Subs/Pubs
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #initalize flags
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        self.r = rospy.Rate(10) #execute at 10 Hz
        self.linear = Vector3(x=0.0,y=0.0,z=0.0) #linear velocities vector. only use x.
        self.angular = Vector3(x=0.0,y=0.0,z=0.0) #angular velocities vector. only use z.

    def getKey(self): #From the website
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def keyToVel(self):
        if self.key == 'i': #forward
            self.linear.x = 1.0
            self.angular.z = 0.0
        if self.key == 'u': #forward turn left
            self.linear.x = 1.0
            self.angular.z = 1.0
        if self.key == 'o': #forward turn right
            self.linear.x = 1.0
            self.angular.z = -1.0
        elif self.key == 'j': #turn counterclockwise
            self.linear.x = 0.0
            self.angular.z = 1.0
        elif self.key == 'l': #turn clockwise
            self.linear.x = 0.0
            self.angular.z = -1.0
        elif self.key == ',': #backwards
            self.linear.x = -1.0
            self.angular.z = 0.0
        elif self.key == 'm': #backwards turn left
            self.linear.x = -1.0
            self.angular.z = -1.0
        elif self.key == '.': #backwards turn right
            self.linear.x = -1.0
            self.angular.z = 1.0
        else:
            self.linear.x = 0.0
            self.angular.z = 0.0

    def run(self):
        print ""
        print "Reading keyboard commands now to drive the Neato."
        print "Use the following grid to drive:"
        print "-------------------------------------------------"
        print "\tu\ti\to"
        print "\tj\tk\tl"
        print "\tm\t,\t."
        print "-------------------------------------------------"
        print "Anything else := stop"
        print 'Ctrl-C to quit'

        while self.key != '\x03': #while you haven't exited via Ctrl-C
            self.key = self.getKey()
            if self.key == '\x03':
                break
            print self.key
            self.keyToVel()
            twist_msg = Twist(linear=self.linear, angular = self.angular)
            print twist_msg
            self.pub.publish(twist_msg)
            self.r.sleep()
        print "Node is finished"

toloooop=Teleop()
toloooop.run()
