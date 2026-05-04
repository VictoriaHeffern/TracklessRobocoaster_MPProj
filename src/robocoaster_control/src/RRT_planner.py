#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
CHASSIS RRT PLANNER

- Manages chassis movement through the obstacle environment
- denotes specific points within environment as targets
- uses the arm_rrt_planner.py file to "talk" to the arm when needed
- uses robocoaster_telemetry.py to calculate foorces
"""

################################################################################################################
#############################
# IMPORTS
#############################
import math
import random
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial import cKDTree
from robocoaster_telemetry import *
from control_msgs.msg import JointTrajectoryControllerState

from arm_rrt_planner import ArmRRTPlanner

# Tuning vars
STEP_SIZE        = 2.0   # metres per RRT
GOAL_BIAS        = 0.15  
MAX_ITER         = 5000  
GOAL_RADIUS      = 2.0   # buffer for goal, not exact pos
EDGE_CHECKS      = 8     # intermediate collision checks per steer step
MAX_PLAN_RETRIES = 3     # attempts to plan for specific goal pos
ARM_SWOOP_EVERY  = 1     # trigger fo arm shwooping idle every N waypoints

# world sampling bounding box
WORLD_X_MIN = -150.0
WORLD_X_MAX =  150.0
WORLD_Y_MIN =  -85.0
WORLD_Y_MAX =   85.0

# Wall geom extracted from model
# offset -0.169006, 2.27396 from origin
_MODEL_OFFSET_X = -0.169006
_MODEL_OFFSET_Y =  2.273960

_RAW_WALLS = [
    # x          y          length      width  yaw
    # Wall_102
    (-11.036,   -59.429,    66.6000,    1.0,  0.000000),
    # Wall_24
    (-51.2449,  -42.3683,   11.0669,    1.0,  0.564552),
    # Wall_25
    ( -6.41622, -39.9302,   82.0421,    1.0, -0.006292),
    # Wall_27
    ( 23.7669,  -24.3601,   32.3899,    1.0,  1.570790),
    # Wall_29
    (-24.237,   -13.787,    39.8504,    1.0,  0.000000),
    # Wall_32
    (-89.992,   -25.493,    22.3496,    1.0, -1.570800),
    # Wall_35
    ( 27.3015,  -62.2685,   14.8505,    1.0, -0.523579),
    # Wall_36
    ( 38.4425,  -70.893,    15.5741,    1.0, -0.787193),
    # Wall_37
    ( 77.886,   -76.055,    69.5998,    1.0,  0.000000),
    # Wall_38
    (112.186,   -69.380,    14.3500,    1.0,  1.570800),
    # Wall_39
    ( 81.261,   -62.705,    62.8500,    1.0,  3.141590),
    # Wall_40
    ( 47.472,   -59.8415,    9.09991,   1.0,  2.356280),
    # Wall_41
    ( 47.737,   -53.849,     9.85015,   1.0,  0.785398),
    # Wall_42
    ( 48.6575,  -49.445,     6.10010,   1.0,  2.618020),
    # Wall_44
    ( 28.9355,  -24.465,    34.1717,    1.0, -1.253900),
    # Wall_49
    ( 68.065,   -21.082,    15.8500,    1.0,  0.000000),
    # Wall_50
    ( 75.489,   -14.282,    14.5996,    1.0,  1.570940),
    # Wall_51
    ( 68.063,    -7.48196,  15.8501,    1.0,  3.141590),
    # Wall_52
    ( 60.6265,  -14.282,    14.6005,    1.0, -1.572490),
    # Wall_55
    ( 32.632,     8.63454,   5.34929,   1.0,  1.308920),
    # Wall_56
    ( 43.4935,   17.814,    25.9937,    1.0,  0.602207),
    # Wall_57
    ( 66.467,    24.893,    26.3500,    1.0,  0.000000),
    # Wall_58
    ( 89.3805,   17.605,    26.1350,    1.0, -0.618617),
    # Wall_59
    (102.932,    -2.04646,  26.5994,    1.0, -1.308980),
    # Wall_60
    (103.159,   -25.929,    24.8507,    1.0, -1.832590),
    # Wall_61
    ( 89.685,   -45.2385,   26.9678,    1.0, -2.498080),
    # Wall_62
    ( 67.123,   -53.029,    25.3500,    1.0,  3.141590),
    # Wall 63
    ( 52.1985,  -51.4415,    7.34996,   1.0,  2.617970),
    # Wall_64 
    ( 49.454,   -49.8537,    0.1600,    0.15, 0.045091),
    # Wall 70
    (-90.018,    19.021,    23.5999,    1.0, -1.570800),
    # Wall 72
    (-22.357,     5.82504,  20.6000,    1.0,  0.000000),
    # Wall 73
    ( -6.75299,  10.404,    15.7854,    1.0,  0.667960),
    # Wall 75
    ( 18.720,    18.992,    38.0062,    1.0, -0.750118),
    # Wall_77
    (-112.186,    8.41654, 136.6910,    1.0, -1.570800),
    # Wall_78
    (-78.011,   -59.429,    69.3500,    1.0,  0.000000),
    # Wall_81
    (-79.882,    53.074,    24.1005,    1.0,  0.000000),
    # Wall 82
    (-68.332,    40.899,    25.3499,    1.0, -1.570800),
    # Wall 83
    (-58.907,    28.724,    19.8505,    1.0,  0.000000),
    # Wall_84
    (-47.657,    21.9145,   15.1001,    1.0, -1.308940),
    # Wall_85
    (-57.242,     8.51754,  27.3504,    1.0, -2.618000),
    # Wall_86
    (-68.652,    -6.49496,  17.8505,    1.0, -1.570800),
    # Wall_88
    (-78.261,    76.262,    68.8500,    1.0,  0.000000),
    # Wall_89
    (-44.336,    61.9165,   29.6910,    1.0, -1.570800),
    # Wall_91
    (-37.325,    52.817,    18.5128,    1.0,  0.642382),
    # Wall_92
    (-21.639,    58.063,    18.3500,    1.0,  0.000000),
    # Wall_93
    ( -6.28699,  53.006,    17.7518,    1.0, -0.648205),
    # Wall 94
    (  2.60301,  39.6905,   18.0999,    1.0, -1.308980),
    # Wall 96
    (-26.3838,   31.316,     7.34959,   1.0, -1.568520),
    # Wall 97
    (-22.9809,   28.141,     7.8210,    1.0,  0.000000),
    # Wall 98
    (-19.5628,   31.316,     7.3500,    1.0,  1.573070),
    # Wall 99
    (-22.988,    34.4835,    7.8357,    1.0, -3.139400),
]


def _build_walls():
    # apply offset to link pos
    walls = []
    cos_cache = []
    sin_cache = []

    for (lx, ly, length, width, yaw) in _RAW_WALLS:
        wx = lx + _MODEL_OFFSET_X
        wy = ly + _MODEL_OFFSET_Y
        walls.append((wx, wy, length, width))
        cos_cache.append(math.cos(-yaw))
        sin_cache.append(math.sin(-yaw))

    return walls, cos_cache, sin_cache

################################################################################################################
#############################
# CLASS for CHASSIS AND NAV
#############################

class RobocoasterRRT(object):

    def __init__(self):
        rospy.init_node("robocoaster_rrt_safe_smooth")

        # build map
        self.walls, self.cos_cache, self.sin_cache = _build_walls()

        # Enter desired goal waypoint coords here
        #self.goals = [(35.0, -55.0), (-51.0, 42.0), (-97.0, 5.9)] 
        #self.goals = [(-97.0, 5.9), (-51.0, 42.0)] 
        self.goals = [(35.0, -55.0)] 


        self.robot_l = 1.75
        self.robot_w = 1.10
        self.margin  = 3.6 # SAFTEY BUFFER!!!!

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        rospy.sleep(1.0)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])

    def is_collision(self, px, py):
        hl = self.robot_l + self.margin
        hw = self.robot_w + self.margin

        for i, (wx, wy, length, width) in enumerate(self.walls):
            dx = px - wx
            dy = py - wy
            c  = self.cos_cache[i]
            s  = self.sin_cache[i]
            
            # FIXED: Standard 2D vector rotation
            lx = dx * c - dy * s
            ly = dx * s + dy * c
            
            half_l = length * 0.5 + hl
            half_w = width  * 0.5 + hw
            if abs(lx) < half_l and abs(ly) < half_w:
                return True
        return False

    def is_edge_collision(self, ax, ay, bx, by):
        # Calculate true distance of the segment
        dist = math.hypot(bx - ax, by - ay)
        
        # Force a check every 0.2 meters, minimum of 1 check
        checks = int(dist / 0.2) + 1 
        
        for k in range(1, checks + 1):
            t = k / float(checks + 1)
            if self.is_collision(ax + t * (bx - ax), ay + t * (by - ay)):
                return True
        return False

    def steer(self, bx, by, rx, ry):
        angle = math.atan2(ry - by, rx - bx)
        return (bx + STEP_SIZE * math.cos(angle),
                by + STEP_SIZE * math.sin(angle))

    def smooth_path(self, path):
        # greedy shortcut
        if len(path) <= 2:
            return path
        improved = True
        while improved:
            improved = False
            i = 0
            smoothed = [path[0]]
            while i < len(path) - 1:
                best_j = i + 1
                for j in range(len(path) - 1, i + 1, -1):
                    ax, ay = smoothed[-1]
                    bx, by = path[j]
                    if not self.is_collision(bx, by) and \
                       not self.is_edge_collision(ax, ay, bx, by):
                        best_j = j
                        improved = True
                        break
                smoothed.append(path[best_j])
                i = best_j
            path = smoothed
        return path

    def plan(self, goal):
        gx, gy = goal
        nodes = [(self.x, self.y)]
        parents = [-1]

        for attempt in range(MAX_PLAN_RETRIES):
            result = self._rrt(nodes, parents, gx, gy)
            if result is not None:
                rospy.loginfo("RRT found path on attempt %d (%d waypoints before smoothing)",
                              attempt + 1, len(result))
                smoothed = self.smooth_path(result)
                rospy.loginfo("Path smoothed to %d waypoints", len(smoothed))
                return smoothed
            rospy.logwarn("RRT attempt %d/%d failed, retrying...", attempt + 1, MAX_PLAN_RETRIES)

        return None

    def _rrt(self, nodes, parents, gx, gy):
        nodes = list(nodes)
        parents = list(parents)

        for _ in range(MAX_ITER):
            rx, ry = (gx, gy) if random.random() < GOAL_BIAS else \
                     (random.uniform(WORLD_X_MIN, WORLD_X_MAX),
                      random.uniform(WORLD_Y_MIN, WORLD_Y_MAX))

            # nearest neighbour 
            tree = cKDTree(nodes)
            _, best_i = tree.query([rx, ry])

            bx, by = nodes[best_i]
            nx, ny = self.steer(bx, by, rx, ry)

            if self.is_collision(nx, ny):
                continue
            if self.is_edge_collision(bx, by, nx, ny):
                continue

            nodes.append((nx, ny))
            parents.append(best_i)

            if (nx - gx) ** 2 + (ny - gy) ** 2 < GOAL_RADIUS ** 2:
                path = []
                cur = len(nodes) - 1
                while cur != -1:
                    path.append(nodes[cur])
                    cur = parents[cur]
                return path[::-1]

        return None

################################################################################################################
##################################################
# CLASS for FULL STACK
##################################################

class RobocoasterFullControl(RobocoasterRRT):

    def __init__(self):
        super(RobocoasterFullControl, self).__init__()

        self.arm_pub = rospy.Publisher(
            'arm_controller/command',
            JointTrajectory, queue_size=10)
        
        self.tracker=GondolaTelemetry()
        
        rospy.sleep(0.5)

        self.arm_planner      = ArmRRTPlanner()
        self.joint_names = ['joint_1', 'joint_2', 'joint_3',
                     'joint_4', 'joint_5', 'joint_6']
        self.current_arm_pose = [0.0] * 6
        self.victory_pose     = [0.0, -0.6, 1.0, 0.0, 0.5, 0.0]

        # find TRUE starting pos
        self.first_pose_set = False
        rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, self.joint_state_cb)
        
        # Wait until pos is recieved
        rospy.loginfo("Waiting for joint states...")
        while not self.first_pose_set and not rospy.is_shutdown():
            # ensure joint_state_cb can actuall function
            rospy.sleep(0.1) 
        rospy.loginfo("Joint states received! Starting mission.")

    def joint_state_cb(self, msg):
        try:
            indices = [msg.joint_names.index(j) for j in self.joint_names]
            self.current_arm_pose = [msg.actual.positions[i] for i in indices]
            self.first_pose_set = True
        except (ValueError, IndexError) as e:
            rospy.logwarn_throttle(2.0, "joint_state_cb error: %s. Got: %s", e, msg.joint_names)

    def send_arm_cmd(self, path, duration=5.0):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.15) 
        
        for i, config in enumerate(path):
            point = JointTrajectoryPoint()
            point.positions = config
            point.time_from_start = rospy.Duration((i + 1) * (duration / len(path)))
            msg.points.append(point)
            
        self.arm_pub.publish(msg)
        self.current_arm_pose = path[-1]

    def drive(self, path):
        MAX_SPEED = 3.5
        MIN_SPEED = 1.5 
        STOP_DISTANCE = 2.0
        rate = rospy.Rate(15)

        last_wp = len(path) - 1

        for wp_idx, (wx, wy) in enumerate(path):
            # TRIGGER ARM SWOOPS (every N waypoints)
            if wp_idx % ARM_SWOOP_EVERY == 0:
                swoop_target = self.arm_planner.get_random_pose()
                swoop_path = self.arm_planner.plan(self.current_arm_pose, swoop_target)
                if swoop_path:
                    self.send_arm_cmd(swoop_path, duration=4.0)

            is_final = (wp_idx == last_wp)

            while not rospy.is_shutdown():
                dx, dy = wx - self.x, wy - self.y
                dist = math.hypot(dx, dy)
                self.tracker.update((self.x, self.y, self.yaw), self.current_arm_pose)
                
                # If close enough, move to next
                if dist < STOP_DISTANCE: 
                    break
                
                target = math.atan2(dy, dx)
                err = math.atan2(math.sin(target - self.yaw), math.cos(target - self.yaw))
                
                cmd = Twist()

                if abs(err) > 1.0:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 1.5 * err
                else:
                    if is_final:
                        cmd.linear.x = MIN_SPEED
                    else:
                        cmd.linear.x = MAX_SPEED

                    if dist < 3.0:
                        cmd.angular.z = 0.4 * err
                    else:
                        cmd.angular.z = 1.5 * err
                self.cmd_pub.publish(cmd)
                rate.sleep()

        # safety stop
        self.cmd_pub.publish(Twist())

    def run(self):
        for i, goal in enumerate(self.goals):
            rospy.loginfo("Navigating to Goal %d", i + 1)
            base_path = self.plan(goal)
            if base_path:
                self.drive(base_path)
                rospy.loginfo("Success! Victory Pose...")
                victory_path = self.arm_planner.plan(self.current_arm_pose, self.victory_pose)
                if victory_path:
                    self.send_arm_cmd(victory_path, duration=3.0)
                rospy.sleep(4.0)
            else:
                rospy.logerr("Base path failed for goal %d", i + 1)
        self.tracker.plot_session()


################################################################################################################
#############################
# MAIN
#############################

if __name__ == "__main__":
    try:
        # FullControl NOT base class!!!!!
        RobocoasterFullControl().run()
    except rospy.ROSInterruptException:
        pass