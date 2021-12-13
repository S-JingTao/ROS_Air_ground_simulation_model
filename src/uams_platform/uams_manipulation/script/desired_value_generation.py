# -*- coding: utf-8 -*-
# @Time    : 2021/1/26:上午10:04
# @Author  : jingtao sun
import numpy as np

class DesiredGen(object):
    def __init__(self):
        # end_effector desired values,respect to world frame

        self.ee_desired_pos = np.array([[0, 0, 0], [3, 6, 10]])
        self.ee_desired_linear_vel = np.array([[0, 0, 0], [0, 0, 0]])
        self.ee_desired_euler = np.array([[0, 0, 0], [0, 0, 0]])
        self.ee_desired_euler_vel = np.array([[0, 0, 0], [0, 0, 0]])

        # uav_base desired euler angles,vel, respect to world frame
        self.base_desired_euler = np.array([[0, 0, 0], [0, 0, 0]])
        self.base_desired_euler_vel = np.array([[0, 0, 0], [0, 0, 0]])
        # uav_base desired altitude,z-vel, respect to world frame
        self.base_desired_altitude = np.array([0, 10.4])
        self.base_desired_altitude_vel = np.array([0, 0])
        # 运行时间区间
        self.desired_time = np.array([0, 20])
        # 在起始位置和最终位置之间进行的采样数目
        self.num_point = 100
        pass

    def desired_calculation(self):

        for i in range(0,3):
            shape = np.shape(self.ee_desired_pos)

        pass
