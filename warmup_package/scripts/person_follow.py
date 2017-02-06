#!/usr/bin/env python

"""Person following node"""

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Twist, Vector3, Quaternion, Point
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from sensor_msgs.msg import LaserScan
from neato_node.msg	import Bump
from nav_msgs.msg import Odometry
import tf
import rospy
import numpy
import math
from math import pi

#Helper functions go here:
def convert_pose_to_xy_and_theta(msg):
    pose = msg.pose.pose
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    # print pose.position.x
    return (pose.position.x, pose.position.y, angles[2])

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return math.atan2(math.sin(z), math.cos(z))

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2

class PersonFollower(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Subscriber('/bump', Bump, self.process_bump)
        rospy.Subscriber('/odom', Odometry, self.processOdom)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.r = rospy.Rate(10)
        self.moves = Twist(linear=Vector3(x = 0.0), angular=Vector3(z = 0.0))
        self.see_person = False
        self.is_bumped = False
        self.pos = Vector3(x=0,y=0,z=0)

    def processOdom(self,msg):
        self.pos.x, self.pos.y, self.pos.z = convert_pose_to_xy_and_theta(msg)

    def process_scan(self,msg):
        self.person = []
        self.see_person = False
        for i in range(0,20):
            dist = msg.ranges[i]
            if dist == 0.0:
                continue
            elif dist < 1.0:
                self.see_person = True
                self.person.append([dist, i*pi/180])
        for i in range(340,360):
            dist = msg.ranges[i]
            if dist == 0.0:
                continue
            elif dist < 1.0:
                self.see_person = True
                self.person.append([dist, (i-360)*pi/180])

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
                self.find_target(self.person)
                #self.moves.linear.x = -.1
                #self.moves.angular.z = -.2/(self.wall[0][0])
                #print "person at:"
                #print str(self.avgangle)
                print "I need to turn:"
                print str(angle_diff(self.pos.z, self.avgangle))
            else:
                self.moves.linear.x = 0.0
                self.moves.angular.z = 0.0

                #print "I'm at:"
                #print str(self.pos.z)
                #self.moves.angular.z = .5
            #self.pub.publish(self.moves)
            self.r.sleep()
        print "Node is finished!"
        self.fucking_stop()

FollowIt = PersonFollower()
FollowIt.run()
