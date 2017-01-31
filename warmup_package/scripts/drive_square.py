#!/usr/bin/env python
"""Attempting to drive a 1mx1m square"""

from geometry_msgs.msg import Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry
import rospy
import tf

#https://github.com/paulruvolo/comprobo15/blob/master/my_pf/scripts/helper_functions.py
def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

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
        self.pos = Vector3(x=0,y=0,th=0)
        self.start_pos = Vector3(x=0,y=0,th=0)
        self.start_turn = 0
        self.time_elapsed = 0 #in seconds
        self.turns = 0 #how many times has turned

    def processOdom(self,msg):
        self.pos.x, self.pos.y, self.pos.th = convert_pose_to_xy_and_theta(msg.pose)
        print self.pos

    def drive_motherfucker(self): #drive forward for one meter, then activate turn function
        self.start_pos.x, self.start_pos.y = self.pos.x, self.pos.y
        self.moves.linear.x = 1.0
        self.moves.angular.z = 0.0
        self.pub.publish(self.moves)
        while ((self.pos.x - self.start_pos.x)**2 + (self.pos.y - self.start_pos.y)**2) < 1: #while have traveled less than a meter
            self.r.sleep()
        if self.distance_covered > 1: #if have driven for more than 1 meter, turn
            self.turn_baby_turn(self)

    def turn_baby_turn(self):
        self.start_pos.th = self.pos.th
        self.moves.linear.x = 0.0
        self.moves.angular.z = 1.0
        self.pub.publish(self.moves)
        while self.pos.th - self.start_pos.th < 90:
            self.r.sleep()
        self.turns += 1
        if self.turns >= 4: #once you've turned four times, you're back where you started, so stop.
            self.fuckingstop()
        else:
            self.drive_motherfucker()

    def fuckingstop(self):
        self.moves.linear.x = 0.0
        self.moves.angular.z = 0.0
        self.pub.publish(self.moves)

    def run(self):
        rospy.on_shutdown(self.fuckingstop)
        while not rospy.is_shutdown():
            self.drive_motherfucker()
            self.r.sleep()
        print "Node is finished"

squaaaaare=DriveSquare()
squaaaaare.run()
