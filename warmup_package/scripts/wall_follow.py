#!/usr/bin/env python

"""Wall following node"""

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from neato_node.msg	import Bump
import rospy

class WallFollower(object):
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
        for dist in msg.ranges[0:10]:
            if dist == 0.0:
                continue
            elif dist < 1.0:
                self.see_obstacle = True
                print "WALL!"
                return
        self.see_wall = False
        print "NO WALL!"
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
            elif not self.see_wall:
                self.moves.linear.x = .1
            else:
                self.moves.angular.z = 3.0
            self.pub.publish(self.moves)
            self.r.sleep()
        print "Node is finished!"
        self.fucking_stop()

FollowIt = WallFollower()
FollowIt.run()
# for dist in msg.ranges[350:361]:
# 	if dist == 0.0:
# 		continue
# 	if dist < 1.0:
# 		self.see_obstacle = True
# 		return
