#!/usr/bin/python
# -*- coding: utf8 -*- 
import argparse
import yaml
import os
import time
import numpy as np
import copy

from ament_index_python.packages import get_package_share_directory

def get_package_dir(package_name):
    share_dir = get_package_share_directory(package_name)
    package_dir = share_dir.replace('install', 'src').removesuffix(f'/share/{package_name}')
    return package_dir

# mode
INIT = 0
TELEOP = 1
TASK_CONTROL = 2
JOINT_CONTROL = 3
AI = 4
MOVEIT = 5
IDLE = 6

mode_dict = {
    0: "INIT",
    1: "TELEOP",
    2: "TASK_CONTROL",
    3: "JOINT_CONTROL",
    4: "AI",
    5: "MOVEIT",
    6: "IDLE",
}
