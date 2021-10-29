#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import rospkg

odom_rec = False
odom_data = None



rospack = rospkg.RosPack()
current_path = rospack.get_path('simulation')
current_path = current_path + "/urdf/"
print(current_path)


payload = open(current_path+"payload.urdf.xacro",'r')

urdf_payload = payload.read()


def odom_callback(data):
    global odom_rec
    global odom_data
    odom_rec = True
    odom_data = data


rospy.init_node('insert_object')
rospy.Subscriber("/dummy0/odom", Odometry, odom_callback)
rospy.wait_for_service('gazebo/spawn_urdf_model')
attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
attach_srv.wait_for_service()
detach_srv.wait_for_service()
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)

while odom_rec==False:
    continue

initial_pose = Pose()
initial_pose.position = odom_data.pose.pose.position
initial_pose.position.z += 0.5
initial_pose.orientation = odom_data.pose.pose.orientation

spawn_model_prox("payload2", urdf_payload, "payload", initial_pose, "world")
req = AttachRequest()
req.model_name_1 = "dummy0"
req.link_name_1 = "dummy0/base_footprint"
req.model_name_2 = "payload2"
req.link_name_2 = "box"
attach_srv.call(req)
