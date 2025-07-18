#!/usr/bin/env python

def resolve_conflicts(paths_dict, min_dist=1.5):
    drone_ids = list(paths_dict.keys())

    while True:
        conflict_found = False
        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                d1, d2 = drone_ids[i], drone_ids[j]
                path1, path2 = paths_dict[d1].poses, paths_dict[d2].poses
                min_len = min(len(path1), len(path2))

                for k in range(min_len):
                    p1 = path1[k].position
                    p2 = path2[k].position
                    dist = ((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)**0.5
                    if dist < min_dist:
                        print(f"Conflict between {d1} and {d2} at index {k}. Delaying {d2}")
                        delay_pose = path2[k]
                        path2.insert(k, delay_pose)
                        paths_dict[d2].poses = path2
                        conflict_found = True
                        break
                if conflict_found:
                    break
            if conflict_found:
                break

        if not conflict_found:
            break

    return paths_dict
