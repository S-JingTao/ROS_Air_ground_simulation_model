#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Time    : 2021/1/7:下午8:58
# @Author  : jingtao sun
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from waypoint_generation_library import WaypointGen

class AdaptationControl(object):
    def __init__(self):
        # self.dlqrPublisher = rospy.Publisher("/uams/command/motor_speed", Actuators, queue_size=1)
        self.dlqrPublisher = rospy.Publisher("/neo11/command/motor_speed", Actuators, queue_size=1)
        self.receivedImuQuat = Quaternion()
        # 六轴的飞机的物理参数
        self.thrustConstant = 1.269e-05
        self.momentConstant = 0.016754
        self.g = 9.8  # [m/s^2]
        self.m = 4.88  # [kg]，包含了机械臂的质量

        self.Ixx = 6.08870e-02  # [kg*m^2]
        self.Iyy = 6.87913e-02  # [kg*m^2]
        self.Izz = 1.48916e-01  # [kg*m^2]
        gamma = self.thrustConstant / self.momentConstant
        self.L = 0.2895  # [m]
        self.PI = 3.14159
        temp_pid = [4, 0.5, 2]
        # 设定自适应PID的参数，总共12个
        self.param_Z = np.array(temp_pid)  # K5p,K5i,K5d
        self.param_roll = np.array([20, 0.1, 11])  # K7p,K7i,K7d
        self.param_pitch = np.array([20, 0.1, 11])  # K9p,K9i,K9d
        self.param_yaw = np.array([5000, 0.1, 160])  # K11p,K11i,K11d

        # position control gains hand-tuned
        # proportional gain
        self.kpPos = np.array(([0.1, 0.1, 1]))
        # derivative gain
        self.kdPos = np.array(([0.1, 0.1, 1]))

        self.speedAllocationMatrix = np.array([[self.thrustConstant, self.thrustConstant, self.thrustConstant,
                                                self.thrustConstant, self.thrustConstant, self.thrustConstant],
                                               [(0.5) * self.L * self.thrustConstant,
                                                self.L * self.thrustConstant,
                                                (0.5) * self.L * self.thrustConstant,
                                                (-0.5) * self.L * self.thrustConstant,
                                                (-1) * self.L * self.thrustConstant,
                                                (-0.5) * self.L * self.thrustConstant],
                                               [-0.5 * (3 ** 0.5) * self.L * self.thrustConstant, 0,
                                                0.5 * (3 ** 0.5) * self.L * self.thrustConstant,
                                                0.5 * (3 ** 0.5) * self.L * self.thrustConstant,
                                                0, -0.5 * (3 ** 0.5) * self.L * self.thrustConstant],
                                               [self.momentConstant, (-1) * self.momentConstant,
                                                self.momentConstant, (-1) * self.momentConstant,
                                                self.momentConstant, (-1) * self.momentConstant]])

        # variable to check whether first pass has been completed to start calculating "dt"
        self.firstPass = False
        # first pass dt corresponding to 100 hz controller
        self.firstPassDt = 0.01
        # time now subtracted by start time
        self.startTime = rospy.get_time()
        # previous time placeholder
        self.prevTime = 0
        # variable to keep track of the previous error in each state
        self.prevRPYErr = np.zeros((3, 1))

        # generate the waypoints
        WaypointGeneration = WaypointGen()
        self.waypoints, self.desVel, self.desAcc, self.timeVec = WaypointGeneration.waypoint_calculation()

        # deadbands [x-pos, y-pos, z-pos, yaw]
        self.waypointDeadband = np.array(([0.3, 0.3, 0.5, 5 * self.PI / 180]))

    def calc_error_integral(self, state):
        cur_time = rospy.get_time() - self.startTime
        # 设置步长
        if not self.firstPass:
            dt = self.firstPassDt
            self.firstPass = True
        else:
            dt = cur_time - self.prevTime

        if dt <= 0.0001:
            dt = 0.01

        nearest_index = np.searchsorted(self.timeVec, cur_time)
        if nearest_index >= np.size(self.timeVec):
            nearest_index = int(np.size(self.timeVec) - 1)

        err_pos = np.array(([self.waypoints[nearest_index, 0] - state[0, 0],
                             self.waypoints[nearest_index, 1] - state[1, 0],
                             self.waypoints[nearest_index, 2] - state[2, 0]]))

        err_vel = np.array(([self.desVel[nearest_index, 0] - state[3, 0],
                             self.desVel[nearest_index, 1] - state[4, 0],
                             self.desVel[nearest_index, 2] - state[5, 0]]))

        desired_line_acc = np.array(
            ([self.desAcc[nearest_index, 0] + self.kpPos[0] * err_pos[0] + self.kdPos[0] * err_vel[0],
              self.desAcc[nearest_index, 1] + self.kpPos[1] * err_pos[1] + self.kdPos[1] * err_vel[1],
              self.desAcc[nearest_index, 2] + self.kpPos[2] * err_pos[2] + self.kdPos[2] * err_vel[2]]))

        desired_rpy = np.array(([(1 / self.g) * (
                    desired_line_acc[0] * np.sin(self.waypoints[nearest_index, 3]) - desired_line_acc[1] * np.cos(
                self.waypoints[nearest_index, 3])),
                                 (1 / self.g) * (desired_line_acc[0] * np.cos(self.waypoints[nearest_index, 3]) +
                                                 desired_line_acc[1] * np.sin(self.waypoints[nearest_index, 3])),
                                 self.waypoints[nearest_index, 3]]))

        err_rpy = np.array(([desired_rpy[0] - state[6, 0],
                             desired_rpy[1] - state[7, 0],
                             desired_rpy[2] - state[8, 0]]))

        err_rpy_vel = np.array(([(err_rpy[0] - self.prevRPYErr[0]) / dt,
                                 (err_rpy[1] - self.prevRPYErr[1]) / dt,
                                 self.desVel[nearest_index, 3] - state[11, 0]]))

        # 计算计算积分项
        integral_err = np.array(([err_pos[2] * dt, err_rpy[0] * dt,
                                  err_rpy[1] * dt, err_rpy[2] * dt]))

        # record the current time
        self.prevTime = cur_time
        # record the current error
        self.prevRPYErr = err_rpy

        # 设置死区 apply deadbands when reaching the final waypoint
        if nearest_index == (np.size(self.timeVec) - 1):
            # x-pos and y-pos deadband check
            if (err_pos[0] <= self.waypointDeadband[0]) and (err_pos[0] >= (-1) * self.waypointDeadband[0]):
                err_pos[0] = 0
            if (err_pos[1] <= self.waypointDeadband[1]) and (err_pos[1] >= (-1) * self.waypointDeadband[1]):
                err_pos[1] = 0
            if (err_pos[2] <= self.waypointDeadband[2]) and (err_pos[2] >= (-1) * self.waypointDeadband[2]):
                err_pos[2] = 0
            # yaw deadband check
            if (err_rpy[2] <= self.waypointDeadband[3]) and (err_rpy[2] >= (-1) * self.waypointDeadband[3]):
                err_rpy[2] = 0

        return err_pos[2], err_vel[2], err_rpy, err_rpy_vel, integral_err, dt

    def ctrl_update(self, state):
        err_pos_z, err_vel_z, err_rpy, err_rpy_vel, integral_err, dt = self.calc_error_integral(state)

        # 获得虚拟控制量
        n0_x = 0
        n0_y = 0
        n0_z = 0

        temp_u2 = ((self.Izz - self.Iyy) / self.Ixx) * (self.prevRPYErr[0] / dt) * state[11, 0]
        temp_u3 = ((self.Ixx - self.Izz) / self.Iyy) * (self.prevRPYErr[1] / dt) * state[11, 0]
        temp_u4 = ((self.Iyy - self.Ixx) / self.Izz) * (self.prevRPYErr[0] / dt) * (self.prevRPYErr[0] / dt)

        u1 = self.m / (np.cos(state[6, 0]) * np.cos(state[7, 0])) * (
                    self.g - self.param_Z[1] * err_pos_z - self.param_Z[0] * (
                        err_vel_z - self.param_Z[1] * integral_err[0] - self.param_Z[0] * err_pos_z) - err_pos_z -
                    self.param_Z[2] * err_vel_z)

        u2 = self.Ixx * (-self.param_roll[1] * err_rpy[0] - self.param_roll[0] * (
                    err_rpy_vel[0] - self.param_roll[1] * integral_err[1] - self.param_roll[0] * err_rpy[0]) + temp_u2 -
                         err_rpy[0] - self.param_roll[2] * err_rpy_vel[0] - n0_x / self.Ixx - self.g / self.Ixx)

        u3 = self.Iyy * (-self.param_pitch[1] * err_rpy[1] - self.param_pitch[0] * (
                err_rpy_vel[1] - self.param_pitch[1] * integral_err[2] - self.param_pitch[0] * err_rpy[1]) + temp_u3 -
                         err_rpy[1] - self.param_pitch[2] * err_rpy_vel[1] - n0_y / self.Iyy - self.g / self.Iyy)

        u4 = self.Izz * (-self.param_yaw[1] * err_rpy[2] - self.param_yaw[0] * (
                err_rpy_vel[2] - self.param_yaw[1] * integral_err[3] - self.param_yaw[0] * err_rpy[2]) + temp_u4 -
                         err_rpy[2] - self.param_yaw[2] * err_rpy_vel[2] - n0_z / self.Izz - self.g / self.Izz)

        desired_input = np.array(([u1], [u2], [u3], [u4]))

        print(desired_input)

        motor_speeds = Actuators()
        motor_speeds.angular_velocities = np.zeros((6, 1))
        motor_speed_transition_vec = np.dot(np.linalg.pinv(self.speedAllocationMatrix), desired_input)
        motor_speeds.angular_velocities = np.sqrt(np.abs(motor_speed_transition_vec).tolist())

        # 发布飞行器电机角速度，控制飞行
        self.dlqrPublisher.publish(motor_speeds)

    def state_update(self, odomInput):
        state = np.zeros((12, 1))
        # position
        state[0] = odomInput.pose.pose.position.x
        state[1] = odomInput.pose.pose.position.y
        state[2] = odomInput.pose.pose.position.z
        # velocity
        state[3] = odomInput.twist.twist.linear.x
        state[4] = odomInput.twist.twist.linear.y
        state[5] = odomInput.twist.twist.linear.z
        # angular position
        [roll, pitch, yaw] = euler_from_quaternion([odomInput.pose.pose.orientation.x,
                                                    odomInput.pose.pose.orientation.y,
                                                    odomInput.pose.pose.orientation.z,
                                                    odomInput.pose.pose.orientation.w])
        state[6] = roll
        state[7] = pitch
        state[8] = yaw

        # angular rate
        state[9] = odomInput.twist.twist.angular.x
        state[10] = odomInput.twist.twist.angular.y
        state[11] = odomInput.twist.twist.angular.z

        # if a nan is seen then set it to 0
        for i in range(0, len(state)):
            if np.isnan(state[i]):
                state[i] = 0

        self.ctrl_update(state)

    def converter(self):
        rospy.Subscriber('/uav_localization/odom', Odometry, self.state_update, queue_size=1)
        rospy.spin()


def main():
    rospy.init_node('uams_ada_control_node', anonymous=False)
    operator = AdaptationControl()
    try:
        operator.converter()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
