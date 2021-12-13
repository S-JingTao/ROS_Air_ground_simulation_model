#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Time    : 2021/1/8:下午5:35
# @Author  : jingtao sun

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import open_manipulator_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from open_manipulator_msgs.srv import *

home_joints = [0, -1.052, 0.354, 0.703]
pre_target1_joints = [0.318, -0.548, 0.769, -0.242]
target1_joints = [0.321, 0.020, 0.314, -0.301]
pre_target2_joints = [1.75027215481, -1.21644675732, 0.639670014381, 0.65961176157]
target2_joints = [1.74567019939, -0.449456393719, 0.845223426819, -0.294524312019]
open_joint = 0.01
close_joint = -0.01

class MoveGroupPythonIntefaceTutorial(object):
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()
        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot’s
        ## kinematic model and the robot’s current joint states
        robot = moveit_commander.RobotCommander()
        ## Instantiate a `PlanningSceneInterface`_ object. This provides a remote interface
        ## for getting, setting, and updating the robot’s internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()
        ## Instantiate a `MoveGroupCommander`_ object. This object is an interface
        ## to a planning group (group of joints). In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group’s name to “panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)
        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
    def go_to_joint_state(self, joint_goal):
        move_group = self.move_group
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()
        time.sleep(3.5)
        return 0

    def control_grapper(self, joint_value):
        rospy.wait_for_service("/open_manipulator/goal_tool_control")
        set_joint_position = rospy.ServiceProxy("/open_manipulator/goal_tool_control", SetJointPosition)
        msg = open_manipulator_msgs.msg.JointPosition()
        msg.joint_name.append("gripper")
        msg.position.append(joint_value)
        resp1 = set_joint_position("arm", msg, 5)
        time.sleep(1.5)

    def pick1(self):
        global home_joints
        global pre_target1_joints
        global target1_joints
        global open_joint
        global close_joint
        self.go_to_joint_state(home_joints)
        self.go_to_joint_state(pre_target1_joints)
        self.control_grapper(open_joint)
        self.go_to_joint_state(target1_joints)
        self.control_grapper(close_joint)
    def place1(self):
        global home_joints
        global pre_target2_joints
        global target2_joints
        self.go_to_joint_state(home_joints)
        self.go_to_joint_state(target2_joints)
        self.control_grapper(open_joint)
        self.go_to_joint_state(pre_target2_joints)
        self.control_grapper(close_joint)
    def pick2(self):
        global home_joints
        global pre_target1_joints
        global target1_joints
        global open_joint
        global close_joint
        self.go_to_joint_state(home_joints)
        self.go_to_joint_state(pre_target2_joints)
        self.control_grapper(open_joint)
        self.go_to_joint_state(target2_joints)
        self.control_grapper(close_joint)
    def place2(self):
        global home_joints
        global pre_target2_joints
        global target2_joints
        self.go_to_joint_state(home_joints)
        self.go_to_joint_state(target1_joints)
        self.control_grapper(open_joint)
        self.go_to_joint_state(pre_target1_joints)
        self.control_grapper(close_joint)

def main():
    try:
        tutorial = MoveGroupPythonIntefaceTutorial()
        while(True):
            tutorial.pick1()
            tutorial.place1()
            tutorial.pick2()
            tutorial.place2()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()