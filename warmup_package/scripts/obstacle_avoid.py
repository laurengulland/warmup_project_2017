#!/usr/bin/env python

"""Obstacle Avoidance Node"""
""" Implemented using finite state control: switches between travelling in original
    direction and following along wall (obstacle) in order to go around obstacles """

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

class ObstacleAvoider(object):
    def __init__(self):
        rospy.init_node('obstacle_avoid')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Subscriber('/bump', Bump, self.process_bump)
        rospy.Subscriber('/odom', Odometry, self.processOdom)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publish neato velocities
        self.r = rospy.Rate(10) #Execute at 10 Hz
        self.pos = Vector3(x=0,y=0,z=0) #current position, as a vector. z is theta here, not z position (lol flying neatos).
        self.start_pos = Vector3(x=0,y=0,z=0)
        self.moves = Twist(linear=Vector3(x = 0.0), angular=Vector3(z = 0.0)) #velocities to publish
        self.state = 'TRAVEL' #TRAVEL or AVOID
        self.see_thing = False
        self.see_morething = False
        self.is_bumped = False

    def processOdom(self,msg): #process odometry inputs into self position.
        self.pos.x, self.pos.y, self.pos.z = convert_pose_to_xy_and_theta(msg)

    def process_scan(self,msg): #process laser scan data. if there's a cluster in
        self.thing = []
        self.see_thing = False
        for i in range(0,40):
            dist = msg.ranges[i]
            if dist == 0.0: #ignore blank scans (glitches)
                continue
            elif dist < 1.0: #if thing is found within a meter, flag its distance and location in self.thing[]
                self.see_thing = True
                self.thing.append([dist, i*pi/180])
        for i in range(320,360):
            dist = msg.ranges[i]
            if dist == 0.0: #ignore blank scans (glitches)
                continue
            elif dist < 1.0: #if thing is found within a meter, flag its distance and location in self.thing[]
                self.see_thing = True
                self.thing.append([dist, (i-360)*pi/180])
        for i in range(270,320): #find objects along side of robot
            dist = msg.ranges[i]
            if dist < 0.25 and dist > 0.0:
                self.see_morething = True
        for i in range(40, 90): #find objects along side of robot
            dist = msg.ranges[i]
            if dist < 0.25 and dist > 0.0:
                self.see_morething = True
        if self.see_thing:
            self.thing.sort(key=lambda lst: lst[0]) #sort lists by angles (second element)
            if self.thing[0][1] > 180: #if the angle is on one side
                self.thing[0][0] *= -1 #flip some shit so it goes the right way

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
        self.r.sleep()
        self.start_pos.z = self.pos.z #set initial position at start of movement.
        while not rospy.is_shutdown():
            if self.is_bumped: #stop when bumped.
                self.fucking_stop()
                break
            elif self.see_thing: #follow the thing proportionally.
                self.moves.linear.x = 0.1
                self.moves.angular.z = -.2/(self.thing[0][0]) #turn to move parallel to the thing proportionally
                self.state = 'AVOID'
            elif not self.see_thing and self.see_morething: #if nothing in front but travelling next to object
                self.state = 'FOLLOW'
                self.moves.linear.x = 0.5 #follow object next to you until it ends
            elif not self.see_thing and not self.see_morething: #if nothing around you
                # print numpy.degrees(self.pos.z)
                # print numpy.degrees(min((2*pi - abs(self.start_pos.z - self.pos.z)), (abs(self.start_pos.z - self.pos.z))))
                if abs(self.start_pos.z - self.pos.z) > pi/16: #if not travelling in original starting direction
                    #return to original direction of travel
                    self.moves.angular.z = 0.1
                    self.state = 'RETURN TO PATH'
                else:
                    #move straight again
                    self.moves.linear.x = 0.5
                    self.state = 'TRAVEL'
            print self.state
            self.pub.publish(self.moves)
            self.r.sleep()
        print "Node is finished!"
        self.fucking_stop()

FollowIt = ObstacleAvoider()
FollowIt.run()
