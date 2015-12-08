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
from sensor_msgs.msg import Range
from functions import JointTrajectory, ScrewTrajectory, CartesianTrajectory

class Trajectory(object):
    def __init__(self, limb):
        self._done = False
        self._state = 0
        
    def set_pos_callback(self, data):
        self._euclidean_goal = data

#        if self._state == 2:
#            self.execute_move(data)

        if self._state == 4:
            self.fine_move(data)
    
    def set_state_callback(self, data):
        self._state = data.data

    def laserscan_callback(self, data):
        self._laserscan = data
    
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
        Xstart = copy(np.asarray(pose))
        pose[0:3,0:3] = R
        pose[0:3,3] = p
        Xend = copy(np.asarray(pose))
        
        # Compute straight-line trajectory for path
        N = 500
        Xlist = CartesianTrajectory(Xstart, Xend, 1, N, 5)
        thList = np.empty((N,7))
        thList[0] = q0;
        
        for i in range(N-1):
        # Solve for joint angles
            seed = 0
            q_ik = kdl_kin.inverse(Xlist[i+1], thList[i])
            while q_ik == None:
                seed += 0.3
                q_ik = kdl_kin.inverse(pose, q0+seed)
            thList[i+1] = q_ik
#            rospy.loginfo(q_ik)
        
#        # Solve for joint angles
#        seed = 0.3
#        q_ik = kdl_kin.inverse(pose, q0+seed)
#        while q_ik == None:
#            seed += 0.3
#            q_ik = kdl_kin.inverse(pose, q0+seed)
#        rospy.loginfo(q_ik)
        
#        q_list = JointTrajectory(q0,q_ik,1,100,5)
        
#        for q in q_list:
            # Format joint angles as limb joint angle assignment      
            angles = limb_interface.joint_angles()
            for ind, joint in enumerate(limb_interface.joint_names()):
                angles[joint] = q_ik[ind]
#            rospy.loginfo(angles)
            rospy.sleep(.003)
            
            # Send joint move command
            limb_interface.set_joint_position_speed(.3)
            limb_interface.set_joint_positions(angles)
        self._done = True
        print('Done')

    def fine_move(self, xstart):
        #set initial position
        print("start")
        xmod = xstart
        zmin = 0.12
        
        while self._laserscan.range > zmin:
            print("while")
            xmod.position.x = xstart.position.x
            xmod.position.y = xstart.position.y

            xmod.orientation.x = xstart.orientation.x
            xmod.orientation.y = xstart.orientation.y
            xmod.orientation.z = xstart.orientation.z
            xmod.orientation.w = xstart.orientation.w

            if self._laserscan.range >= self._laserscan.max_range:
                xmod.position.z = xmod.position.z - 0.1
            else:
                if (self._laserscan.range - zmin) > 0.05:
                    xmod.position.z = xmod.position.z - self._laserscan.range + zmin
                else:
                    break
                    
            self.execute_move(xmod)
            
        pub_state = rospy.Publisher('state', Int16, queue_size = 10, latch=True)
        rospy.loginfo(5) 
        rospy.sleep(.2)
        pub_state.publish(5)          
                          
        self._done = True
        print('Done')
        
        
def main():
    rospy.init_node('move_to_laser')
    traj = Trajectory('right')
    rospy.Subscriber("hand_position", Pose, traj.set_pos_callback)
    rospy.Subscriber("state", Int16, traj.set_state_callback)
    rospy.Subscriber("/robot/range/right_hand_range/state", Range, traj.laserscan_callback)
    rospy.loginfo('In loop')
    rospy.spin()   
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        
