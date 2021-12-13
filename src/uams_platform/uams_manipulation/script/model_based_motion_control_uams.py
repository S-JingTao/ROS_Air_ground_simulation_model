#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Time    : 2021/1/24:下午5:36
# @Author  : jingtao sun
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class MotionControl(object):
    def __init__(self):
        pass

    def uav_state_update(self, odomInput):

        uav_cur_state = np.zeros((12, 1))
        # 当前的position
        uav_cur_state[0] = odomInput.pose.pose.position.x
        uav_cur_state[1] = odomInput.pose.pose.position.y
        uav_cur_state[2] = odomInput.pose.pose.position.z
        # 当前的 linear velocities
        uav_cur_state[3] = odomInput.twist.twist.linear.x
        uav_cur_state[4] = odomInput.twist.twist.linear.y
        uav_cur_state[5] = odomInput.twist.twist.linear.z
        # 当前的angular
        [roll, pitch, yaw] = euler_from_quaternion([odomInput.pose.pose.orientation.x,
                                                    odomInput.pose.pose.orientation.y,
                                                    odomInput.pose.pose.orientation.z,
                                                    odomInput.pose.pose.orientation.w])
        uav_cur_state[6] = roll
        uav_cur_state[7] = pitch
        uav_cur_state[8] = yaw
        # 当前的 angular rate
        uav_cur_state[9] = odomInput.twist.twist.angular.x
        uav_cur_state[10] = odomInput.twist.twist.angular.y
        uav_cur_state[11] = odomInput.twist.twist.angular.z
        # 处理NAN值
        uav_cur_state = self.handle_nan_value(uav_cur_state)


        self.uav_ctrl_update(uav_cur_state)

        pass

    def handle_nan_value(self, state):
        for i in range(0, len(state)):
            if np.isnan(state[i]):
                state[i] = 0
        return state

    def manipulator_ctrl_update(self, msg):

        arm_cur_position_state = np.zeros((4, 1))
        arm_cur_velocity_state = np.zeros((4, 1))
        arm_name = []

        for i in range(2, len(msg.position)):
            arm_name.append(msg.name[i])

        arm_cur_position_state[0] = msg.position[2]
        arm_cur_position_state[1] = msg.position[3]
        arm_cur_position_state[2] = msg.position[4]
        arm_cur_position_state[3] = msg.position[5]

        arm_cur_velocity_state[0] = msg.position[2]
        arm_cur_velocity_state[1] = msg.position[3]
        arm_cur_velocity_state[2] = msg.position[4]
        arm_cur_velocity_state[3] = msg.position[5]

        rospy.loginfo('name: %s', str(arm_name))
        rospy.loginfo('joint %.2f, %.2f,%.2f,%.2f,',
                      arm_cur_position_state[0],
                      arm_cur_position_state[1],
                      arm_cur_position_state[2],
                      arm_cur_position_state[3])

        rospy.loginfo('vel %.2f, %.2f,%.2f,%.2f,',
                      arm_cur_velocity_state[0],
                      arm_cur_velocity_state[1],
                      arm_cur_velocity_state[2],
                      arm_cur_velocity_state[3])

        arm_cur_position_state = self.handle_nan_value(arm_cur_position_state)
        arm_cur_velocity_state = self.handle_nan_value(arm_cur_velocity_state)

        pass

    def uav_ctrl_update(self, state):
        pass

    def data_converter(self):
        # 订阅UAV和manipulator的所有状态量
        # 订阅UAV姿态融合解算之后的状态量
        rospy.Subscriber("/uav_localization/odom", Odometry, self.uav_state_update, queue_size=1)
        # 订阅机械臂的各关节状态量
        rospy.Subscriber("/joint_states", JointState, self.manipulator_ctrl_update)
        rospy.spin()
        pass


def main():
    rospy.init_node('uams_motion_control_node', anonymous=False)
    control = MotionControl()

    try:
        control.data_converter()
    except rospy.ROSInterruptException:
        pass

    pass


if __name__ == '__main__':
    main()
