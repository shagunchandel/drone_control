import numpy as np
from geometry_msgs.msg import PoseArray

def detect_conflict(path1, path2, min_dist=1.5):
    length = min(len(path1.poses), len(path2.poses))
    for i in range(length):
        p1 = path1.poses[i].position
        p2 = path2.poses[i].position
        dist = np.linalg.norm([p1.x - p2.x, p1.y - p2.y, p1.z - p2.z])
        if dist < min_dist:
            return True, i
    return False, -1

def resolve_conflicts(paths_dict):
    keys = list(paths_dict.keys())
    for i in range(len(keys)):
        for j in range(i+1, len(keys)):
            p1 = paths_dict[keys[i]]
            p2 = paths_dict[keys[j]]
            conflict, idx = detect_conflict(p1, p2)
            if conflict:
                print(f"Conflict at step {idx} between {keys[i]} and {keys[j]}")
                # e.g., delay drone2
                p2.poses = [p2.poses[0]] + p2.poses
                paths_dict[keys[j]] = p2
    return paths_dict
