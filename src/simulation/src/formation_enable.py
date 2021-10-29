import rospy
import cv2
import time
import rospkg
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist,PoseStamped,TransformStamped
from scipy.ndimage.morphology import distance_transform_edt as bwdist
from scipy.ndimage.morphology import grey_dilation
from scipy.spatial import distance
from geometry_msgs.msg import PoseStamped
from tracking_pid.msg import FollowPathActionResult
from tracking_pid.msg import traj_point
from std_srvs.srv import SetBool
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

formation_points = None

def odom_callback(data):
    global formation_points
    current_odom = data
    t_a_b = TransformStamped()
    t_a_b.header.stamp = rospy.Time.now()
    t_a_b.transform.translation.x = current_odom.pose.pose.position.x
    t_a_b.transform.translation.y = current_odom.pose.pose.position.y
    t_a_b.transform.translation.z = current_odom.pose.pose.position.z
    t_a_b.transform.rotation.x = current_odom.pose.pose.orientation.x
    t_a_b.transform.rotation.y = current_odom.pose.pose.orientation.y
    t_a_b.transform.rotation.z = current_odom.pose.pose.orientation.z
    t_a_b.transform.rotation.w = current_odom.pose.pose.orientation.w
    formation_points = []
    formation = [[+1.0,-1.0],
                 [+0.0,-1.0],
                 [-1.0,-1.0],
                 [+1.0,+1.0],
                 [+0.0,+1.0],
                 [-1.0,+1.0]]
    for i in range(len(formation)):
        tt = PoseStamped()
        tt.header.stamp=rospy.Time.now()
        tt.pose.position.x= formation[i][0]
        tt.pose.position.y= formation[i][1]
        tt.pose.orientation.w = 1.0
        formation_points.append(tf2_geometry_msgs.do_transform_pose(tt, t_a_b))

rospy.init_node('formation_enable')
rospy.Subscriber("/dummy0/odom", Odometry, odom_callback)
trajectory_ser = []
trajectory_ser.append(rospy.ServiceProxy('/robot0/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot1/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot2/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot3/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot4/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot5/enable_control', SetBool))

goal_clients = []
goal_clients.append(actionlib.SimpleActionClient('/robot0/move_base',MoveBaseAction))
goal_clients.append(actionlib.SimpleActionClient('/robot1/move_base',MoveBaseAction))
goal_clients.append(actionlib.SimpleActionClient('/robot2/move_base',MoveBaseAction))
goal_clients.append(actionlib.SimpleActionClient('/robot3/move_base',MoveBaseAction))
goal_clients.append(actionlib.SimpleActionClient('/robot4/move_base',MoveBaseAction))
goal_clients.append(actionlib.SimpleActionClient('/robot5/move_base',MoveBaseAction))

for i in range(len(goal_clients)):
    goal_clients[i].wait_for_server()

while(type(formation_points) == type(None)):
    continue

for i in range(len(goal_clients)):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = formation_points[i].pose.position.x
    goal.target_pose.pose.position.y = formation_points[i].pose.position.y
    goal.target_pose.pose.orientation.w = 1.0
    goal_clients[i].send_goal(goal)

for i in range(len(goal_clients)):
    wait = goal_clients[i].wait_for_result()
    print(goal_clients[i].get_result())
    
for i in range(len(trajectory_ser)):
    print(trajectory_ser[i].call(True))
