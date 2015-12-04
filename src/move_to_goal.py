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
        self._state = 0
        
#    def set_pos_callback(self, data):
#        self._euclidean_goal = data
#        if self._state == 5:
#            self.execute_move(data)
    
    def set_state_callback(self, data):
        rospy.loginfo(data.data)
        self._state = data.data
        if self._state == 5:
            self.execute_move(data)
    
    def execute_move(self, pos):
        right_gripper = baxter_interface.Gripper('right')
        right_gripper.close()
        rospy.loginfo('moving')
  
        # Send joint move command
        robot = URDF.from_parameter_server()
        base_link = robot.get_root()
        kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')
        limb_interface = baxter_interface.limb.Limb('right')
        angles = limb_interface.joint_angles()
        q_goal = [-.315, -1.019, .3064, 1.5286, -.4912, .5844, -2.7899]
        q0 = kdl_kin.random_joint_angles()
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        for ind in range(len(q0)):
            q0[ind] = current_angles[ind]
        
        q_list = JointTrajectory(q0,np.asarray(q_goal),1,10,5)
        for q in q_list:
            for ind, joint in enumerate(limb_interface.joint_names()):
                angles[joint] = q[ind]
            limb_interface.set_joint_positions(angles)
            rospy.sleep(.3)
            pub_state = rospy.Publisher('state', Int16, queue_size = 10)
            
        right_gripper.open()
        rospy.loginfo(2)
        pub_state.publish(2)
        self._done = True
        print('Done')
        
        
def main():
    rospy.init_node('move_to_goal')
    traj = Trajectory('right')
#    rospy.Subscriber("block_position", Pose, traj.set_pos_callback)
    rospy.Subscriber("state", Int16, traj.set_state_callback)
    rospy.loginfo('In loop')
    rospy.spin()   
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        