#! /usr/bin/env python3
# -*- coding:UTF-8 -*-
# Author:jingtaosun
# time: 2020.12.14
import rospy,sys
import moveit_commander
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Pose


class AddScene:
	def __init__(self):
		# 初始化move_group的API
		moveit_commander.roscpp_initialize(sys.argv)
		# 初始化ROS节点
		rospy.init_node('scene_add_project_demo')
		# 初始化场景对象
		scene = PlanningSceneInterface()
		# 场景中移除物体指令：
		scene.remove_world_object('table')
		#scene.remove_attached_object(end_effector_link,'tool')
	    # 设置桌面并添加进场景
		rospy.sleep(2)
    	
		table_size = [2,2,0.01]
		table_pose = PoseStamped()
		table_pose.header.frame_id = 'base_link'
		table_pose.pose.position.x = 0
		table_pose.pose.position.y = 0
		table_pose.pose.position.z = -0.005
		table_pose.pose.orientation.x = 0.0
		table_pose.pose.orientation.y = 0.0
		table_pose.pose.orientation.z = 0.0
		table_pose.pose.orientation.w = 1.0
		scene.add_box('table', table_pose, table_size)
	    
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)

if __name__ == '__main__':
	AddScene()
