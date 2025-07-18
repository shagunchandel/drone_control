#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseArray
from drone_control.msg import FollowPathAction, FollowPathGoal
from conflict_resolver import resolve_conflicts

def create_path(x_offset=0.0, y_offset=0.0):
    path = PoseArray()
    for i in range(5):
        pose = Pose()
        pose.position.x = i + x_offset
        pose.position.y = y_offset
        pose.position.z = 2.0
        path.poses.append(pose)
    return path

def send_path(drone_id, total_time, pose_array):
    client = actionlib.SimpleActionClient(f"/{drone_id}/follow_path", FollowPathAction)
    client.wait_for_server()

    goal = FollowPathGoal()
    goal.drone_id = drone_id
    goal.path = pose_array
    goal.total_time = total_time

    client.send_goal(goal)
    rospy.loginfo(f"Sent path to {drone_id}")
    client.wait_for_result()
    rospy.loginfo(f"{drone_id} result: {client.get_result()}")

if __name__ == "__main__":
    rospy.init_node("multi_drone_path_planner")

    total_time = 10.0

    # Original paths
    paths = {
        "uav0": create_path(x_offset=0),
        "uav2": create_path(x_offset=2),  # potentially conflicting
    }

    # Resolve conflicts
    resolved_paths = resolve_conflicts(paths)

    # Send paths
    for drone_id, path in resolved_paths.items():
        send_path(drone_id, total_time, path)
