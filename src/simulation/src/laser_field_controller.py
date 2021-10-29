#! /usr/bin/env python3

import rospy
import PID
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,PoseStamped
from math import sin,cos
from nav_msgs.msg import Odometry
from tf import TransformListener
import tf2_ros
import tf2_geometry_msgs #import the packages first

message_pub = None
selfodom = None
goalodom = None
robot_name = None
pid_X = PID.PID(1.25, 0.001, 0.5)
pid_Y = PID.PID(1.25, 0.001, 0.5)
pid_X.setSampleTime(0.1)
pid_Y.setSampleTime(0.1)

def odomCallback(msg):
    global goalodom 
    global robot_name
    test = PoseStamped()
    test.header.frame_id = "map"
    test.header.stamp = rospy.Time.now()
    test.pose = msg.pose.pose
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0)) #tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    if (robot_name != None):
        transform = tf_buffer.lookup_transform(str(robot_name)+"/base_footprint",
                                    test.header.frame_id, #source frame
                                    rospy.Time(0), #get the tf at first available time
                                    rospy.Duration(1.0))
        goalodom = tf2_geometry_msgs.do_transform_pose(test, transform)

def selfOdomCallback(msg):
    global selfodom 
    selfodom = msg

def callback(msg):
    global message_pub
    global selfodom
    global goalodom
    global pid_X
    global pid_Y
    if goalodom != None and selfodom != None:
        cmd = Twist()
        pid_X.SetPoint = 0.0
        pid_Y.SetPoint = 0.0
        pid_X.update(goalodom.pose.position.x)
        pid_Y.update(goalodom.pose.position.y)
        cmd.linear.x = -pid_X.output
        cmd.linear.y = -pid_Y.output
        if (min(msg.ranges) < 1.50):
            beam = msg.ranges.index(min(msg.ranges))
            angle = beam*msg.angle_increment
            cmd.linear.x += cos(angle) * 3.5
            cmd.linear.y += sin(angle) * 3.5
        message_pub.publish(cmd)
        

message_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
rospy.init_node('scan_potential_field')
robot_name = rospy.get_param('~name', "robot0")
sub = rospy.Subscriber('scan', LaserScan, callback,queue_size=1)
sub = rospy.Subscriber('odom', Odometry, selfOdomCallback,queue_size=1)
sub = rospy.Subscriber('/dummy0/odom', Odometry, odomCallback,queue_size=1)
rospy.spin()