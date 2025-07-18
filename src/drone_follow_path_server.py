#!/usr/bin/python3
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, PoseArray
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from drone_control.msg import FollowPathAction, FollowPathFeedback, FollowPathResult

class DronePathFollower:
    def __init__(self, drone_id):
        rospy.init_node(f"{drone_id}_path_server")
        self.drone_id = drone_id
        # MAVROS setup
        self.state = None
        rospy.Subscriber(f"/{drone_id}/mavros/state", State, self.state_cb)
        rospy.wait_for_service(f"/{drone_id}/mavros/cmd/arming")
        rospy.wait_for_service(f"/{drone_id}/mavros/set_mode")
        self.arming_client = rospy.ServiceProxy(f"/{drone_id}/mavros/cmd/arming", CommandBool)
        self.mode_client = rospy.ServiceProxy(f"/{drone_id}/mavros/set_mode", SetMode)
        rospy.Subscriber(f"/{drone_id}/mavros/local_position/pose", PoseStamped, self.pose_cb)
        self.pub_sp = rospy.Publisher(f"/{drone_id}/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.current_pose = None

        self.server = actionlib.SimpleActionServer(f"/{drone_id}/follow_path", FollowPathAction, self.execute, False)
        self.server.start()
        rospy.loginfo(f"[{drone_id}] Server started")

    def state_cb(self, msg): self.state = msg
    def pose_cb(self, msg): self.current_pose = msg.pose

    def setup_offboard(self):
        rate = rospy.Rate(20)
        # publish current pose to prep OFFBOARD
        while not rospy.is_shutdown() and (not self.current_pose or not self.state.connected):
            rospy.sleep(0.1)
        sp = PoseStamped()
        sp.header.frame_id = "map"
        sp.pose = self.current_pose
        for _ in range(100): sp.header.stamp = rospy.Time.now(); self.pub_sp.publish(sp); rate.sleep()

        self.mode_client(0, "OFFBOARD")
        self.arming_client(True)
        rospy.loginfo(f"[{self.drone_id}] OFFBOARD & armed")

    def execute(self, goal):
        if not self.current_pose: 
            self.server.set_aborted(FollowPathResult(False, "No pose"))
            return
        self.setup_offboard()
        path, total_time = goal.path.poses, goal.total_time
        dt = total_time / len(path)
        for i, p in enumerate(path):
            if self.server.is_preempt_requested():
                self.server.set_preempted(FollowPathResult(False, "Preempted")); return
            sp = PoseStamped(header=rospy.Header(frame_id="map", stamp=rospy.Time.now()), pose=p)
            t_end = rospy.Time.now() + rospy.Duration(dt)
            while rospy.Time.now() < t_end:
                self.pub_sp.publish(sp)
                rospy.sleep(0.05)
            self.server.publish_feedback(FollowPathFeedback(current_pose=p, progress_percent=(i+1)/len(path)*100))
        self.server.set_succeeded(FollowPathResult(True, "Done"))

if __name__ == "__main__":
    DronePathFollower(rospy.get_param("~drone_id","uav0"))
    rospy.spin()
