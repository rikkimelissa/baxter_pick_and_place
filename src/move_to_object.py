#!/usr/bin/env python

import rospy
import baxter_interface
from copy import copy
from baxter_interface import CHECK_VERSION
from geometry_msgs.msg import Pose
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from quat import quat_to_so3
import numpy as np
from std_msgs.msg import Int16
from functions import JointTrajectory

class Trajectory(object):
    def __init__(self, limb):
        self._done = False
        self._state = 2
        
    def set_pos_callback(self, data):
        self._euclidean_goal = data
        if self._state == 2:
            self.execute_move(data)
    
    def set_state_callback(self, data):
        self._state = data.data
    
    def execute_move(self, pos):
        rospy.loginfo('moving')
        # Read in pose data
        q = [pos.orientation.w, pos.orientation.x, pos.orientation.y, pos.orientation.z]
        p =[[pos.position.x],[pos.position.y],[pos.position.z]]
        # Convert quaternion data to rotation matrix
        R = quat_to_so3(q);
        # Form transformation matrix
        robot = URDF.from_parameter_server()
        base_link = robot.get_root()
        kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')
        # Create seed with current position
        q0 = kdl_kin.random_joint_angles()
        limb_interface = baxter_interface.limb.Limb('right')
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        for ind in range(len(q0)):
            q0[ind] = current_angles[ind]
        pose = kdl_kin.forward(q0)
        pose[0:3,0:3] = R
        pose[0:3,3] = p
        # Solve for joint angles
        seed = 0.3
        q_ik = kdl_kin.inverse(pose, q0+seed)
        while q_ik == None:
            seed += 0.3
            q_ik = kdl_kin.inverse(pose, q0+seed)
        rospy.loginfo(q_ik)
        
        q_list = JointTrajectory(q0,q_ik,1,100,5)
        
        for q in q_list:
            # Format joint angles as limb joint angle assignment      
            angles = limb_interface.joint_angles()
            for ind, joint in enumerate(limb_interface.joint_names()):
                angles[joint] = q[ind]
            rospy.loginfo(angles)
            
            # Send joint move command
            limb_interface.set_joint_position_speed(1)
            limb_interface.set_joint_positions(angles)
        self._done = True
        print('Done')
        
        
def main():
    rospy.init_node('move_to_object')
    traj = Trajectory('right')
    rospy.Subscriber("block_position", Pose, traj.set_pos_callback)
    rospy.Subscriber("state", Int16, traj.set_state_callback)
    rospy.loginfo('In loop')
    rospy.spin()   
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        
