#!/usr/bin/env python

"""Wall following node"""

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Twist, Vector3, Quaternion, Point
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from sensor_msgs.msg import LaserScan
from neato_node.msg	import Bump
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import rospy
from math import pi
import math

def convert_pose_to_xy_and_theta(msg):
    pose = msg.pose.pose
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

class Sphero(object):
    def __init__(self):
        self.pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.r = rospy.Rate(10)
        self.spherey = Marker()
        self.spherey.header.frame_id = "/odom"
        self.spherey.header.stamp    = rospy.get_rostime()
        self.spherey.ns = "robot"
        self.spherey.id = 0
        self.spherey.type = 2 # sphere
        self.spherey.action = 0
        self.spherey.pose.position.x = 1.0
        self.spherey.pose.position.y = 2.0
        self.spherey.scale.x = 0.5
        self.spherey.scale.y = 0.5
        self.spherey.scale.z = 0.5

        self.spherey.color.r = 0.0
        self.spherey.color.g = 1.0
        self.spherey.color.b = 1.0
        self.spherey.color.a = 1.0

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
        self.pos = Vector3(x=0,y=0,z=0)
        self.marker = Sphero()
        self.direction = 1
        rospy.Subscriber('/odom', Odometry, self.processOdom)

    def make_marker(self,polar_position):
        delta_y = polar_position[0]*math.sin(polar_position[1])
        delta_x= polar_position[0]*math.cos(polar_position[1])
        # print 'x:'
        # print str(delta_x)
        # print 'y:'
        # print str(delta_y)
        self.marker.spherey.pose.position.x = self.pos.x-delta_x
        self.marker.spherey.pose.position.y = self.pos.y+delta_y
    def processOdom(self,msg):
        self.pos.x, self.pos.y, self.pos.z = convert_pose_to_xy_and_theta(msg)
        #self.marker.spherey.pose.position.x = self.pos.x
        #self.marker.spherey.pose.position.y = self.pos.y
    def process_scan(self,msg):
        self.wall = []
        self.see_wall = False
        for i in range(0,20):
            dist = msg.ranges[i]
            if dist == 0.0:
                continue
            elif dist < 1.0:
                self.see_wall = True
                self.wall.append([dist, i])
        for i in range(340,360):
            dist = msg.ranges[i]
            if dist == 0.0:
                continue
            elif dist < 1.0:
                self.see_wall = True
                self.wall.append([dist, i])
        #print str(self.see_wall)
        if self.see_wall:
            self.wall.sort(key=lambda lst: lst[0])
            self.make_marker(self.wall[0])
            self.direction = 1
            if self.wall[0][1] > 180:
                #self.wall[0][0] *= -1
                self.direction = -1



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
            #print 'running!'
            if self.is_bumped:
                self.fucking_stop()
                break
            elif self.see_wall:
                #self.moves.linear.x = -.1
                self.moves.angular.z = -.2/(self.wall[0][0])*self.direction
            else:
                self.moves.linear.x = .1
                self.moves.angular.z = 0.0
                #self.moves.angular.z = .5
            self.pub.publish(self.moves)
            self.marker.pub.publish(self.marker.spherey)
            self.r.sleep()
        print "Node is finished!"
        self.fucking_stop()

FollowIt = WallFollower()
FollowIt.run()
