#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ARM RRT PLANNER

- Handles logic for the KUKA arm
- the EE must be EE_MIN_Z metres above to be valid
- EE must face forwards (i.e. over the diff drive wheels) to be valid
- FK from the joint origins in kr560_r3100_2_macro.xacro
"""

#############################
# IMPORTS
#############################

import math
import random
import numpy as np

#############################
# CONSTANTS
#############################

# from kr560_r3100_2_macro.xacro
HARDWARE_LIMITS = [
    (-3.22885, 3.22885), 
    (-2.26890, 0.34900),  
    (-1.74500, 2.87900), 
    (-6.10800, 6.10800), 
    (-2.09400, 2.09400),  
    (-6.10800, 6.10800), 
]

# Min EE height above base 
EE_MIN_Z = 0.5

# Min X forward
# Keeps arm mass over the drive wheels, NOT the rear caster!!!!!1
EE_MIN_X = 0.5

GOAL_POSE = [0.0, -0.8, 1.5, 0.0, 0.3, 0.0]

#############################
# FK Things
#############################

def _rz(angle):
    # 4x4 homog rot matrix Z
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [ c, -s, 0, 0],
        [ s,  c, 0, 0],
        [ 0,  0, 1, 0],
        [ 0,  0, 0, 1],
    ], dtype=float)

def _rx(angle):
    # 4x4 homog rot matrix X
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [1,  0,  0, 0],
        [0,  c, -s, 0],
        [0,  s,  c, 0],
        [0,  0,  0, 1],
    ], dtype=float)

def _t(x, y, z):
    # 4x4 homog trans matrix X
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1],
    ], dtype=float)

def forward_kinematics(joints):
    """
    EE pos relative to arm base_link
    From <origin> tags in kr560_r3100_2_macro.xacro
    Returns (x, y, z) in the arm base frame
    """
    j1, j2, j3, j4, j5, j6 = joints

    T = np.dot(_rx(-math.pi), _rz(j1))

    T = np.dot(np.dot(np.dot(T, _t(0.5, 0, -0.86)), _rx(math.pi / 2)), _rz(j2))

    T = np.dot(np.dot(T, _t(1.55, 0, 0)), np.dot(_rz(-math.pi / 2), _rz(j3)))

    T = np.dot(np.dot(np.dot(T, _t(0.18, 0, 0)), _rx(math.pi / 2)), _rz(j4))

    T = np.dot(np.dot(np.dot(T, _t(0, 0, -1.0345)), _rx(-math.pi / 2)), _rz(j5))

    T = np.dot(T, np.dot(_rx(math.pi / 2), _rz(j6)))

    T = np.dot(T, _t(0, 0, -0.305))

    return T[0, 3], T[1, 3], T[2, 3]

#############################
# CLASS
#############################

class ArmRRTPlanner(object):
    def __init__(self):
        self.hardware_limits = HARDWARE_LIMITS
        self.goal_pose       = GOAL_POSE

    def is_valid(self, joints):
        """
        Valid if:
          1. All joints within hardware limits.
          2. EEpos Z >= EE_MIN_Z  
          3. EE pos X >= EE_MIN_X  
        """
        for i, val in enumerate(joints):
            lo, hi = self.hardware_limits[i]
            if not (lo <= val <= hi):
                return False

        x, _, z = forward_kinematics(joints)

        if z < EE_MIN_Z:
            return False

        if x < EE_MIN_X:
            return False

        return True

    def get_random_pose(self):
        # andom valid config for arm swoops
        for _ in range(1000):
            pose = [random.uniform(lo, hi) for lo, hi in self.hardware_limits]
            if self.is_valid(pose):
                return pose
        # Fallback: known-safe forward pose. FK: x=3.14, y=0, z=0.72
        return [0.0, -0.3, 0.8, 0.0, 0.5, 0.0]

    def plan(self, start, goal, max_iter=1500):
        # RRT 6D joint space. 
        goal = list(goal)
        nodes = [list(start) + [-1]]

        for _ in range(max_iter):
            if random.random() < 0.15:
                target = goal
            else:
                target = [random.uniform(lo, hi) for lo, hi in self.hardware_limits]

            target_arr = np.array(target)

            idx = min(
                range(len(nodes)),
                key=lambda i: np.linalg.norm(np.array(nodes[i][:6]) - target_arr)
            )
            nearest = np.array(nodes[idx][:6])

            diff = target_arr - nearest
            dist = np.linalg.norm(diff)
            if dist == 0:
                continue

            new_config = nearest + (diff / dist) * min(dist, 0.2)

            if not self.is_valid(new_config):
                continue

            nodes.append(list(new_config) + [idx])

            if np.linalg.norm(new_config - np.array(goal)) < 0.25:
                path = []
                curr = len(nodes) - 1
                while curr != -1:
                    path.append(nodes[curr][:6])
                    curr = nodes[curr][6]
                return path[::-1]

        return None

    def plan_to_goal_pose(self, start, max_iter=1500):
        # Plans from start tO  GOAL
        return self.plan(start, self.goal_pose, max_iter=max_iter)