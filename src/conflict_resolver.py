#!/usr/bin/python3

from geometry_msgs.msg import PoseArray
import copy
import math

def euclidean_distance(p1, p2):
    return math.sqrt(
        (p1.x - p2.x)**2 +
        (p1.y - p2.y)**2 +
        (p1.z - p2.z)**2
    )

def resolve_conflicts(paths_dict, min_dist=1.5):
    """
    Adjust drone paths by inserting wait steps to avoid conflicts.
    paths_dict: dict of drone_id -> PoseArray
    min_dist: minimum distance required between drones (meters)
    """
    drone_ids = list(paths_dict.keys())

    # Ensure paths have the same length by padding shorter ones
    max_len = max(len(paths_dict[d].poses) for d in drone_ids)
    for d in drone_ids:
        path = paths_dict[d].poses
        if len(path) < max_len:
            last_pose = path[-1]
            for _ in range(max_len - len(path)):
                path.append(copy.deepcopy(last_pose))
        paths_dict[d].poses = path

    while True:
        conflict_found = False

        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                d1, d2 = drone_ids[i], drone_ids[j]
                path1, path2 = paths_dict[d1].poses, paths_dict[d2].poses

                min_path_len = min(len(path1), len(path2))

                for k in range(min_path_len):
                    dist = euclidean_distance(path1[k].position, path2[k].position)
                    if dist < min_dist:
                        print(f"[ConflictResolver] Conflict between {d1} and {d2} at step {k} (distance = {dist:.2f})")

                        # Delay drone with shorter path (or arbitrarily second)
                        delayed_path = paths_dict[d2].poses
                        delay_pose = copy.deepcopy(delayed_path[k])
                        delayed_path.insert(k, delay_pose)
                        conflict_found = True
                        break

                if conflict_found:
                    break
            if conflict_found:
                break

        if not conflict_found:
            break

    return paths_dict
