#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
G-FORCE AND ACCEL PLOTTER

- Examines forces over time
- Preforms plotting at the end

"""

import numpy as np
import matplotlib.pyplot as plt
import time
from arm_rrt_planner import forward_kinematics

class GondolaTelemetry:
    def __init__(self):
        # Storage data
        self.times = []
        self.accels = []
        self.g_forces = []
        
        # Const
        self.g = 9.81
        self.arm_base_height = 0.65  # 0.4 (chassis) + 0.25 (mount)[cite: 3]
        
        # robot's; state
        self.last_pos = None
        self.last_vel = None
        self.last_time = None

    def update(self, chassis_pose, arm_joints):
        """
        calculates and store instantaneous forces...
        chassis_pose: (x, y, yaw)
        arm_joints: [j1, j2, j3, j4, j5, j6]
        """
        now = time.time()
        cx, cy, cyaw = chassis_pose
        
        # 1. Get Gondola/EE pos in World Frame

        # EE to arm base
        ax, ay, az = forward_kinematics(arm_joints)
        
        # Rot by chassis yaw and  pos
        wx = cx + (ax * np.cos(cyaw) - ay * np.sin(cyaw))
        wy = cy + (ax * np.sin(cyaw) + ay * np.cos(cyaw))
        wz = az + self.arm_base_height
        current_pos = np.array([wx, wy, wz])

        if self.last_time is not None:
            dt = now - self.last_time
            if dt <= 0: return

            # 2. v = delta p / delta t
            current_vel = (current_pos - self.last_pos) / dt
            
            if self.last_vel is not None:
                # 3. a = delta v /  delta t
                accel_vec = (current_vel - self.last_vel) / dt
                accel_mag = np.linalg.norm(accel_vec)
                
                # 4. G-Force
                # Vector sum of motion acceleration and gravity
                total_force_vec = accel_vec + np.array([0, 0, self.g])
                g_mag = np.linalg.norm(total_force_vec) / self.g
                
                # Store data
                self.times.append(now)
                self.accels.append(accel_mag)
                self.g_forces.append(g_mag)

            self.last_vel = current_vel

        self.last_pos = current_pos
        self.last_time = now

    def plot_session(self):
        # plot all data collected
        if not self.times:
            print("No data recorded.")
            return

        t = np.array(self.times) - self.times[0]
        
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 1, 1)
        plt.plot(t, self.accels, 'r-', label='Linear Acceleration')
        plt.ylabel(u'm/s^2')
        plt.title('Gondola Force Profile (Start to End)')
        plt.grid(True)
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(t, self.g_forces, 'g-', label='Resultant G-Force')
        plt.ylabel('G')
        plt.xlabel('Time (s)')
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()