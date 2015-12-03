#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
                
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
print robot.get_group_names()
group = moveit_commander.MoveGroupCommander("left_arm")

