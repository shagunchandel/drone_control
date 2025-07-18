#!/usr/bin/env python

import rospy
import actionlib
from drone_control.msg import FollowPathAction, FollowPathFeedback, FollowPathResult
from geometry_msgs.msg import PoseStamped, PoseArray, Twist

class DronePathFollower:
    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.pose_topic = f"/{drone_id}/mavros/local_position/pose"
        self.cmd_vel_topic = f"/{drone_id}/mavros/setpoint_velocity/cmd_vel"

        self.server = actionlib.SimpleActionServer(
            f"/{drone_id}/follow_path", FollowPathAction, self.execute, False)

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)
        self.current_pose = None

        self.server.start()
        rospy.loginfo(f"[{self.drone_id}] Action server ready.")

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def move_to_pose(self, target_pose, duration):
        rospy.sleep(duration)  # Replace with OFFBOARD control for real usage

    def execute(self, goal):
        path = goal.path.poses
        total_time = goal.total_time
        result = FollowPathResult()
        feedback = FollowPathFeedback()

        if not path or not self.current_pose:
            result.success = False
            result.message = "Invalid path or pose unavailable"
            self.server.set_aborted(result)
            return

        step_time = total_time / len(path)
        for i, pose in enumerate(path):
            if self.server.is_preempt_requested():
                result.success = False
                result.message = "Preempted"
                self.server.set_preempted(result)
                return

            self.move_to_pose(pose, step_time)
            feedback.current_pose = pose
            feedback.progress_percent = (i + 1) / float(len(path)) * 100.0
            self.server.publish_feedback(feedback)

        result.success = True
        result.message = "Path execution complete"
        self.server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("drone_path_follower")
    drone_id = rospy.get_param("~drone_id", "uav0")
    DronePathFollower(drone_id)
    rospy.spin()
