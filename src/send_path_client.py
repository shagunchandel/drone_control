#!/usr/bin/python3

import rospy
import actionlib
from drone_control.msg import FollowPathAction, FollowPathGoal
from geometry_msgs.msg import PoseArray, Pose

def build_path():
    path = PoseArray()
    for i in range(5):
        pose = Pose()
        pose.position.x = i * 1.0
        pose.position.y = 0.0
        pose.position.z = 2.0
        path.poses.append(pose)
    return path

def send_path(drone_id, total_time):
    client = actionlib.SimpleActionClient(f"/{drone_id}/follow_path", FollowPathAction)
    client.wait_for_server()

    goal = FollowPathGoal()
    goal.drone_id = drone_id
    goal.path = build_path()
    goal.total_time = total_time

    client.send_goal(goal)
    client.wait_for_result()
    print(client.get_result())

if __name__ == '__main__':
    rospy.init_node("send_path_client")
    send_path("uav0", 10.0)
