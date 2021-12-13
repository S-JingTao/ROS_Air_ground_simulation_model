#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
# Author:jingtaosun
# time: 2020.12.28

import rospy
import sys
import moveit_commander

class UamsArmMoveit:
	def __init__(self):
		# 初始化move_group的API,底层是通过C++进行实现的
		moveit_commander.roscpp_initialize(sys.argv)
		# 初始化节点
		rospy.init_node('arm_moveit', anonymous=True)
		# 是否需要使用笛卡尔空间的运动规划，获取参数，如果没有设置，则默认为True，即走直线
		cartesian = rospy.get_param('~cartesian', True)
		# 初始化所需要控制的机械臂和夹爪的规划组arm_group
		arm = moveit_commander.MoveGroupCommander("arm")
		gripper = moveit_commander.MoveGroupCommander('hand')
		# 当运动规划失败后，允许重新规划
		arm.allow_replanning(True)
		# 设置目标位置所使用的参考坐标系
		arm.set_pose_reference_frame('base_link')
		# 设置机械臂和夹爪运动的允许误差值，单位弧度
		arm.set_goal_joint_tolerance(0.001)
		gripper.set_goal_joint_tolerance(0.001)
		# 设置允许的最大速度和加速度，范围0-1
		arm.set_max_acceleration_scaling_factor(0.5)
		arm.set_max_velocity_scaling_factor(0.5)

		# 控制机械臂回到初始位置
		arm.set_named_target('arm_init')
		# 让机械臂先规划，再运动，阻塞指令，直到机械臂到达home后再向下执行
		arm.go()
		rospy.sleep(1)

		# 设置机械臂的目标位置,并规划执行
		joint_positions = [0.391410, -0.676384, -0.376217, 0.0]
		arm.set_joint_value_target(joint_positions)
		arm.go()
		rospy.sleep(1)

		# 控制机械臂先回到初始化位置
		arm.set_named_target('home')
		arm.go()
		rospy.sleep(1)

		# 关闭并退出moveit
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)

if __name__ == '__main__':
	try:
		UamsArmMoveit()
	except rospy.ROSInterruptException:
		pass