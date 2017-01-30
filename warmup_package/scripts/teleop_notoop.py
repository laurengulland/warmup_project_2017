#!/usr/bin/env/ python
from geometry_msgs.msg import Twist
import rospy
import tty
import select
import sys
import termios

print 'initializing'


#initialize node
rospy.init_node('teleop')
#initiate Subs/Pubs
rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#initalize flags
self.settings = termios.tcgetattr(sys.stdin)
self.key = None
self.r = rospy.Rate(10) #execute at 10 Hz
self.vel = None

def getKey(): #From the website
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

    def keyToVel():
        if self.key == 'i': #forward
            vel = []
        elif self.key == 'k': #stop
            vel = []
        elif self.key == 'j': #turn counterclockwise
            vel = []
        elif self.key == 'l': #turn clockwise
            vel = []
        elif self.key == ',': #go backwards
            vel = []


    def run(self):
        print 'running'
        while self.key != '\x03': #while you haven't exited via Ctrl-C
            print 'not dying'
            self.key = getKey()
            print self.key

            # linear_msg = Vector3(x = 0)
            # angular_msg = Vector3(z = 0)
            # twist_msg = Twist(linear=linear_msg, angular = angular_msg)
            # if self.see_obstacle or self.is_bumped:
            #     twist.msg.linear.x = 0.0
            # else:
            #     twist_msg.linear.x = 1
            # self.pub.publish(twist_msg)
            # print twist_msg
            # self.r.sleep()

            self.r.sleep()
        print "Node is finished"
print 'ending'
