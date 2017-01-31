#!/usr/bin/env python
"""Attempting to drive a 1mx1m square"""

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Twist, Vector3, Quaternion, Point
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from nav_msgs.msg import Odometry
from neato_node.msg	import Bump
import rospy
import tf

#https://github.com/paulruvolo/comprobo15/blob/master/my_pf/scripts/helper_functions.py
def convert_pose_to_xy_and_theta(msg):
    pose = msg.pose.pose
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
        self.is_bumped = False
        self.moves = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0))
        self.pos = Vector3(x=0,y=0,z=0)
        self.start_pos = Vector3(x=0,y=0,z=0)
        self.start_turn = 0
        self.time_elapsed = 0 #in seconds
        self.turns = 0 #how many times has turned

    def processOdom(self,msg):
        self.pos.x, self.pos.y, self.pos.z = convert_pose_to_xy_and_theta(msg)
        # print self.pos
    def process_bump(self,msg):
        if (msg.leftFront==1 or msg.leftFront==1 or msg.rightFront ==1 or msg.rightSide==1):
            print 'BUMP!'
            self.is_bumped = True
        else:
			self.is_bumped = False

    def drive_motherfucker(self): #drive forward for one meter, then activate turn function
        print 'DRIVE!'
        self.start_pos.x, self.start_pos.y = self.pos.x, self.pos.y
        self.moves.linear.x = 1.0
        self.moves.angular.z = 0.0
        print self.moves
        self.pub.publish(self.moves)
        while ((self.pos.x - self.start_pos.x)**2 + (self.pos.y - self.start_pos.y)**2) < 1: #while have traveled less than a meter
            if self.is_bumped: #stop if bumped
                self.fucking_stop()
                break
            self.r.sleep()
        if self.distance_covered > 1: #if have driven for more than 1 meter, turn
            self.turn_baby_turn(self)

    def turn_baby_turn(self):
        print "TURN!"
        self.start_pos.z = self.pos.z
        self.moves.linear.x = 0.0
        self.moves.angular.z = 1.0
        self.pub.publish(self.moves)
        while self.pos.z - self.start_pos.z < 90:
            if self.is_bumped: #stop if bumped
                self.fucking_stop()
                break
            self.r.sleep()
        self.turns += 1
        if self.turns >= 4: #once you've turned four times, you're back where you started, so stop.
            self.fuckingstop()
        else:
            self.drive_motherfucker()

    def fuckingstop(self):
        print 'STOP!'
        self.moves.linear.x = 0.0
        self.moves.angular.z = 0.0
        self.pub.publish(self.moves)
        print self.moves

    def run(self):
        rospy.on_shutdown(self.fuckingstop)
        while not rospy.is_shutdown():
            print 'START!'
            self.drive_motherfucker()
            self.r.sleep()
        print "Node is finished"

squaaaaare=DriveSquare()
squaaaaare.run()
