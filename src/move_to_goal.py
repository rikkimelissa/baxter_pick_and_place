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
    # Initalize class instance
    def __init__(self, limb):
        self._done = False
        self._state = 0
        self._right_gripper = baxter_interface.Gripper('right')
        self._goal = 0
        self._pub_state = rospy.Publisher('state', Int16, queue_size = 10, latch=True)

    
    # Changes the state in the state machine
    def set_state_callback(self, data):
        self._state = data.data
        if self._state == 5:
            self.execute_move(data)
    
    # Sets goal to sort to base on object ID
    def set_goal_callback(self, data):
        self._goal = data.data;
    
    # Controls the grippers and drops off block
    def execute_move(self, pos):

        # Close the gripper
        self._right_gripper.close()
        self._right_gripper.close()
        self._right_gripper.close()
        self._right_gripper.close()
        self._right_gripper.close()
        rospy.loginfo('moving')
  
        # Get robot parameters and create a limb interface instance
        robot = URDF.from_parameter_server()
        base_link = robot.get_root()
        kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')
        limb_interface = baxter_interface.limb.Limb('right')
        angles = limb_interface.joint_angles()
        limb_interface.set_joint_speed(.3)
        
        
        # Set the dropoff position to be dropoff #1
        if self._goal == 1:
            q_goal = [-.675, -.445, 1.626, 1.1336, -1.457, 1.6145, -2.190]
        # Dropoff #2
        else:
            q_goal = [-.066, -.068, 1.738, .8022, -2.23, .917, -2.9057]

        # Create the desired joint trajectory with a quintic time scaling
        q0 = kdl_kin.random_joint_angles()
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        for ind in range(len(q0)):
            q0[ind] = current_angles[ind]
        q_list = JointTrajectory(q0,np.asarray(q_goal),1,50,5)
        for q in q_list:
            for ind, joint in enumerate(limb_interface.joint_names()):
                angles[joint] = q[ind]
            limb_interface.set_joint_positions(angles)
            rospy.sleep(.1)
##        
#        q_0 = q_goal
#        # dropoff position
#        q_goal = [-.675, -.445, 1.626, 1.1336, -1.457, 1.6145, -2.190]
#        q_list = JointTrajectory(np.asarray(q0),np.asarray(q_goal),1,50,5)
#        for q in q_list:
#            for ind, joint in enumerate(limb_interface.joint_names()):
#                angles[joint] = q[ind]
#            limb_interface.set_joint_positions(angles)
#            rospy.sleep(.1)
        
        # Open the gripper
        self._right_gripper.open()
        self._right_gripper.open()
        self._right_gripper.open()
        self._right_gripper.open()
        self._right_gripper.open()
        
        # Set the position to the old goal and the new goal to the desired camera position for seeing the blocks         
        q0 = q_goal
        q_goal = [-.01859, -.5119, 1.7909, 1.232, -1.030, 1.945, -1.31]  
        q_list = JointTrajectory(np.asarray(q0),np.asarray(q_goal),1,50,5)
        for q in q_list:
            for ind, joint in enumerate(limb_interface.joint_names()):
                angles[joint] = q[ind]
            limb_interface.set_joint_positions(angles)
            rospy.sleep(.1)
        
        # Publish next state               
        self._pub_state.publish(1)
        self._done = True
        print('Done')
        
        
def main():
    # Initialize node, subscribers, and class instance
    rospy.init_node('move_to_goal')
    traj = Trajectory('right')
    rospy.Subscriber("state", Int16, traj.set_state_callback)
    rospy.Subscriber("goal", Int16, traj.set_goal_callback)
    rospy.loginfo('In loop')
    rospy.spin()   
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        
