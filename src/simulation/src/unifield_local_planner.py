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
import actionlib
from std_srvs.srv import SetBool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

current_odom = None
trajectory_pub = None
odom_pub = None
rr = None
malfunction = False

def odom_callback(data):
    global current_odom
    global malfunction
    global odom_pub
    global trajectory_pub
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
        if malfunction and i == 5:
            tt = PoseStamped()
            tt.header.stamp=rospy.Time.now()
            tt.pose.position.x= formation[i][0]
            tt.pose.position.y= formation[i][1]
            tt.pose.orientation.w = 1.0
            formation_points.append(tf2_geometry_msgs.do_transform_pose(tt, t_a_b))
            ans = formation_points[i]
            cmd = traj_point()
            cmd.pose.header.frame_id = "map"
            cmd.pose.pose.position = ans.pose.position
            cmd.pose.pose.orientation.w = 1.0
            trajectory_pub[-1].publish(cmd)
            break
        tt = PoseStamped()
        tt.header.stamp=rospy.Time.now()
        tt.pose.position.x= formation[i][0]
        tt.pose.position.y= formation[i][1]
        tt.pose.orientation.w = 1.0
        formation_points.append(tf2_geometry_msgs.do_transform_pose(tt, t_a_b))
        ans = formation_points[i]
        cmd = traj_point()
        cmd.pose.header.frame_id = "map"
        cmd.pose.pose.position = ans.pose.position
        cmd.pose.pose.orientation.w = 1.0
        trajectory_pub[i].publish(cmd)
        


def goal_callback(data):
    global goal_clients
    global trajectory_ser
    global malfunction
    global rr
    t_a_b = TransformStamped()
    t_a_b.header.stamp = rospy.Time.now()
    t_a_b.transform.translation.x = data.pose.position.x
    t_a_b.transform.translation.y = data.pose.position.y
    t_a_b.transform.translation.z = data.pose.position.z
    t_a_b.transform.rotation.x = data.pose.orientation.x
    t_a_b.transform.rotation.y = data.pose.orientation.y
    t_a_b.transform.rotation.z = data.pose.orientation.z
    t_a_b.transform.rotation.w = data.pose.orientation.w
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

    for i in range(len(goal_clients)-2):
        print("i = " +str(i))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = formation_points[i].pose.position
        goal.target_pose.pose.orientation = formation_points[i].pose.orientation
        goal_clients[i].send_goal(goal)
    print(len(formation_points))
    print(len(goal_clients))
    goal1 = MoveBaseGoal()
    goal1.target_pose.header.frame_id = "map"
    goal1.target_pose.header.stamp = rospy.Time.now()
    goal1.target_pose.pose.position = formation_points[-1].pose.position
    goal1.target_pose.pose.orientation = formation_points[-1].pose.orientation
    goal_clients[-1].send_goal(goal1)
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = data.pose.position
    goal.target_pose.pose.orientation = data.pose.orientation
    rr.send_goal(goal)
    
    for i in range(len(goal_clients)):
        wait = goal_clients[i].wait_for_result()
        print(goal_clients[i].get_result())
    wait = rr.wait_for_result()
    print(rr.get_result())
    
    for i in range(len(trajectory_ser)-2):
        print(trajectory_ser[i].call(True))
    trajectory_ser[-1].call(True)
    malfunction = True


rospy.init_node('commonLocalPlanner')
rospy.Subscriber("/dummy0/odom", Odometry, odom_callback)
rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)
rr = actionlib.SimpleActionClient('/dummy0/move_base',MoveBaseAction)
trajectory_pub = []
trajectory_pub.append(rospy.Publisher("/robot0/trajectory", traj_point, queue_size=1))
trajectory_pub.append(rospy.Publisher("/robot1/trajectory", traj_point, queue_size=1))
trajectory_pub.append(rospy.Publisher("/robot2/trajectory", traj_point, queue_size=1))
trajectory_pub.append(rospy.Publisher("/robot3/trajectory", traj_point, queue_size=1))
trajectory_pub.append(rospy.Publisher("/robot4/trajectory", traj_point, queue_size=1))
trajectory_pub.append(rospy.Publisher("/robot5/trajectory", traj_point, queue_size=1))
trajectory_pub.append(rospy.Publisher("/robot6/trajectory", traj_point, queue_size=1))
goal_clients = []
goal_clients.append(actionlib.SimpleActionClient('/robot0/move_base',MoveBaseAction))
goal_clients.append(actionlib.SimpleActionClient('/robot1/move_base',MoveBaseAction))
goal_clients.append(actionlib.SimpleActionClient('/robot2/move_base',MoveBaseAction))
goal_clients.append(actionlib.SimpleActionClient('/robot3/move_base',MoveBaseAction))
goal_clients.append(actionlib.SimpleActionClient('/robot4/move_base',MoveBaseAction))
goal_clients.append(actionlib.SimpleActionClient('/robot5/move_base',MoveBaseAction))
goal_clients.append(actionlib.SimpleActionClient('/robot6/move_base',MoveBaseAction))
trajectory_ser = []
trajectory_ser.append(rospy.ServiceProxy('/robot0/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot1/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot2/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot3/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot4/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot5/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot6/enable_control', SetBool))
for i in range(len(goal_clients)):
    goal_clients[i].wait_for_server()
rospy.spin()