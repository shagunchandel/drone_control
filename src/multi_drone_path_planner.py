#!/usr/bin/python3

import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseArray
from drone_control.msg import FollowPathAction, FollowPathGoal
from conflict_resolver import resolve_conflicts

def create_s_path(offset=0.0, reverse=False):
    """Create an S-shaped path offset in Y, with optional direction reversal"""
    path = PoseArray()
    s_points = [
        (0, 0),
        (1, 1),
        (2, -1),
        (3, 1),
        (4, -1),
        (5, 0)
    ]

    if reverse:
        s_points = list(reversed(s_points))

    for x, y in s_points:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y + offset  # offset in Y to separate start positions
        pose.position.z = 2.0
        path.poses.append(pose)

    return path

def send_path(drone_id, total_time, pose_array):
    client = actionlib.SimpleActionClient(f"/{drone_id}/follow_path", FollowPathAction)
    rospy.loginfo(f"[{drone_id}] Waiting for action server...")
    client.wait_for_server()

    goal = FollowPathGoal()
    goal.drone_id = drone_id
    goal.total_time = total_time
    goal.path = pose_array

    rospy.loginfo(f"[{drone_id}] Sending path with {len(pose_array.poses)} waypoints.")
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo(f"[{drone_id}] Result: {client.get_result()}")

if __name__ == "__main__":
    rospy.init_node("multi_drone_path_planner")

    total_time = 12.0  # seconds

    # Step 1: Generate S-shaped conflicting paths
    paths = {
        "uav0": create_s_path(offset=0.0, reverse=False),
        "uav2": create_s_path(offset=0.0, reverse=True)  # Intentionally crossing uav0
    }

    # Step 2: Resolve conflicts (insert delays)
    resolved_paths = resolve_conflicts(paths, min_dist=1.5)

    # Step 3: Send adjusted paths
    for drone_id, path in resolved_paths.items():
        send_path(drone_id, total_time, path)
