#!/usr/bin/env python

"""Person following node"""

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from neato_node.msg	import Bump
import rospy
import numpy

class PersonFollower(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Subscriber('/bump', Bump, self.process_bump)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.r = rospy.Rate(10)
        self.moves = Twist(linear=Vector3(x = 0.0), angular=Vector3(z = 0.0))
        self.see_wall = False
        self.is_bumped = False

    def process_scan(self,msg):
        self.person = []
        self.see_person = False
        for i in range(0,20):
            dist = msg.ranges[i]
            if dist == 0.0:
                continue
            elif dist < 1.0:
                self.person.append([dist, i])
        for i in range(340,360):
            dist = msg.ranges[i]
            if dist == 0.0:
                continue
            elif dist < 1.0:
                self.person.append([dist, i-360])
        self.find_target(self.person)
        # if self.see_wall:
        #     self.wall.sort(key=lambda lst: lst[0])
        #     if self.wall[0][1] > 180:
        #         self.wall[0][0] *= -1

    def find_target(self, positions):
        self.distances = []
        self.angles = []
        for pose in positions:
            self.distances.append(pose[0])
            self.angles.append(pose[1])
        self.avgdist = numpy.mean(self.distances)
        self.avgangle = numpy.mean(self.angles)


    def process_bump(self,msg):
        if (msg.leftFront==1 or msg.leftFront==1 or msg.rightFront ==1 or msg.rightSide==1):
            print 'BUMP!'
            self.is_bumped = True

        else:
			self.is_bumped = False
    def fucking_stop(self):
        print 'STOP!'
        self.moves.linear.x = 0.0
        self.moves.angular.z = 0.0
        self.pub.publish(self.moves)
    def run(self):
        rospy.on_shutdown(self.fucking_stop)
        while not rospy.is_shutdown():
            if self.is_bumped:
                self.fucking_stop()
                break
            elif self.see_person:
                #self.moves.linear.x = -.1
                self.moves.angular.z = -.2/(self.wall[0][0])
            else:
                self.moves.linear.x = 0.0
                self.moves.angular.z = 0.0
                #self.moves.angular.z = .5
            self.pub.publish(self.moves)
            self.r.sleep()
        print "Node is finished!"
        self.fucking_stop()

FollowIt = WallFollower()
FollowIt.run()
