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
from functions import JointTrajectory, ScrewTrajectory, CartesianTrajectory

class Trajectory(object):
    # Initalize class instance
    def __init__(self, limb):
        self._done = False
        self._state = 0
        self._pub_hand = rospy.Publisher('hand_position', Pose, queue_size = 10, latch=True)
        self._pub_state = rospy.Publisher('state', Int16, queue_size = 10, latch=True)
    
    # Reads in desired pose data    
    def set_pos_callback(self, data):
        self._euclidean_goal = data
        rospy.loginfo(data)
        if self._state == 2:
            self.execute_move(data)
    
    # Changes the state in the state machine
    def set_state_callback(self, data):
        self._state = data.data
    
    # Moves to the desired pose
    def execute_move(self, pos):
        rospy.loginfo('moving')
        # Read in pose data. Adjust the height to be above the block and the length so that the laser sees the table instead of the block
        pos.position.z += .1
        pos.position.x += .005
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
        limb_interface.set_joint_position_speed(.3)
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        for ind in range(len(q0)):
            q0[ind] = current_angles[ind]
        pose = kdl_kin.forward(q0)
        pose[0:3,0:3] = R
        pose[0:3,3] = p
        
        # Solve for joint angles, iterating if no solution is found
        seed = 0.3
        q_ik = kdl_kin.inverse(pose, q0+seed)
        while q_ik == None:
            seed += 0.3
            q_ik = kdl_kin.inverse(pose, q0+seed)
        rospy.loginfo(q_ik)
        
        # Calculate the joint trajectory with a quintic time scaling
        q_list = JointTrajectory(q0,q_ik,1,50,5)
        
        # Iterate through list
        for q in q_list:
            # Format joint angles as limb joint angle assignment      
            angles = limb_interface.joint_angles()
            for ind, joint in enumerate(limb_interface.joint_names()):
                angles[joint] = q[ind]
            rospy.sleep(.07)
            
            # Send joint move command
            limb_interface.set_joint_positions(angles)
        
        # Publish state and hand position

        rospy.sleep(1)
        rospy.loginfo(4) 
        self._pub_state.publish(4)                    
        rospy.loginfo(pos)
        self._pub_hand.publish(pos)  

        self._done = True
        print('Done')
        
        
def main():
    # Initialize node, subscribers, and class instance
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
        
