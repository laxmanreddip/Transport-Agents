#!/usr/bin/env python3
import rospy
import actionlib
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(goal):
    client = actionlib.SimpleActionClient('robot0/move_base',MoveBaseAction)
    client.wait_for_server()
    client.send_goal_and_wait(goal)
    wait = client.wait_for_result()
    print(wait)
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

    
if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        goal = MoveBaseGoal()
        goal.target_pose.header.seq = 0
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 7.07
        goal.target_pose.pose.position.y = -1.91
        goal.target_pose.pose.orientation.w = 1.0
        result = movebase_client(goal)
        if result:
            rospy.loginfo("Goal execution done!")
        time.sleep(5)
        goal = MoveBaseGoal()
        goal.target_pose.header.seq = 1
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 7.19
        goal.target_pose.pose.position.y = 6.97
        goal.target_pose.pose.orientation.w = 1.0
        result = movebase_client(goal)
        if result:
            rospy.loginfo("Goal execution done!")
        time.sleep(5)
        goal = MoveBaseGoal()
        goal.target_pose.header.seq = 2
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 7.60
        goal.target_pose.pose.position.y = -7.83
        goal.target_pose.pose.orientation.w = 1.0
        result = movebase_client(goal)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")