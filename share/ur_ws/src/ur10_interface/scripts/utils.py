import os
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import numpy as np

def get_package_dir(package_name):
    share_dir = get_package_share_directory(package_name)
    package_dir = share_dir.replace('install', 'src').removesuffix(f'/share/{package_name}')
    return package_dir

def get_file_dir(package_name, file_name):
    pkg_dir = get_package_dir(package_name)
    file_dir = os.path.join(pkg_dir, file_name)
    return file_dir

def compress_with_indices(data):
    compressed = [data[0]]
    indices = [[0]]
    
    for idx in range(1, len(data)):
        if data[idx] == False:
            compressed.append(data[idx])
            indices.append([idx])
        elif data[idx] == True and compressed[-1] == False:
            compressed.append(data[idx])
            indices.append([idx])
        elif data[idx] == True and compressed[-1] == True:
            indices[-1].append(idx)
    return compressed, indices


def list_to_pose_stamped(pose_list, frame_id="base_link"):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id

    pose_stamped.pose.position.x = pose_list[0]
    pose_stamped.pose.position.y = pose_list[1]
    pose_stamped.pose.position.z = pose_list[2]

    pose_stamped.pose.orientation.x = pose_list[3]
    pose_stamped.pose.orientation.y = pose_list[4]
    pose_stamped.pose.orientation.z = pose_list[5]
    pose_stamped.pose.orientation.w = pose_list[6]

    return pose_stamped