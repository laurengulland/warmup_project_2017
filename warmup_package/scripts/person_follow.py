#!/usr/bin/env python

"""Person following node"""

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Twist, Vector3, Quaternion, Point
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from neato_node.msg	import Bump
from nav_msgs.msg import Odometry
import tf
import rospy
import numpy
import math
from math import pi
#from marker import Sphero

#Helper functions go here:
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


class PersonFollower(object):
    def __init__(self):
        rospy.init_node('person_follow')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Subscriber('/bump', Bump, self.process_bump)
        self.marker = Sphero()
        rospy.Subscriber('/odom', Odometry, self.processOdom)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.r = rospy.Rate(10)
        self.moves = Twist(linear=Vector3(x = 0.0), angular=Vector3(z = 0.0))
        self.see_person = False
        self.is_bumped = False
        self.pos = Vector3(x=0,y=0,z=0)



    def processOdom(self,msg):
        self.pos.x, self.pos.y, self.pos.z = convert_pose_to_xy_and_theta(msg)
        #self.marker.spherey.pose.position.x = self.pos.x
        #self.marker.spherey.pose.position.y = self.pos.y

    def make_marker(self,polar_position):
        delta_y = polar_position[0]*math.sin(polar_position[1])
        delta_x= polar_position[0]*math.cos(polar_position[1])
        # print 'x:'
        # print str(delta_x)
        # print 'y:'
        # print str(delta_y)
        self.marker.spherey.pose.position.x = self.pos.x-delta_x
        self.marker.spherey.pose.position.y = self.pos.y-delta_y

    def process_scan(self,msg):
        self.person = []
        self.see_person = False
        for i in range(0,40):
            dist = msg.ranges[i]
            if dist == 0.0:
                continue
            elif dist < 1.0:
                self.see_person = True
                self.person.append([dist, i*pi/180])
        for i in range(320,360):
            dist = msg.ranges[i]
            if dist == 0.0:
                continue
            elif dist < 1.0:
                self.see_person = True
                self.person.append([dist, (i-360)*pi/180])

    def find_target(self, positions):
        self.distances = []
        self.angles = []
        for pose in positions:
            self.distances.append(pose[0])
            self.angles.append(pose[1])
        self.avgdist = numpy.mean(self.distances)
        self.avgangle = numpy.mean(self.angles)
        self.make_marker([self.avgdist, self.avgangle])

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
                self.find_target(self.person)
                self.moves.linear.x = 0.2*self.avgdist
                self.moves.angular.z = self.avgangle

            else:
                self.moves.linear.x = 0.2
                self.moves.angular.z = 0.0

            self.pub.publish(self.moves)
            self.marker.pub.publish(self.marker.spherey)
            self.r.sleep()
        print "Node is finished!"
        self.fucking_stop()

FollowIt = PersonFollower()
FollowIt.run()
