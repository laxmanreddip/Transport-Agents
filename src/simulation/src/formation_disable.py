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

rospy.init_node('formation_disable')
trajectory_ser = []
trajectory_ser.append(rospy.ServiceProxy('/robot0/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot1/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot2/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot3/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot4/enable_control', SetBool))
trajectory_ser.append(rospy.ServiceProxy('/robot5/enable_control', SetBool))


for i in range(len(trajectory_ser)):
    print(trajectory_ser[i].call(False))

print("formation disabled")