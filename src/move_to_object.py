#!/usr/bin/env python

import rospy
import baxter_interface
from copy import copy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib
from baxter_interface import CHECK_VERSION
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
from geometry_msgs.msg import Pose
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from quat import quat_to_so3
import numpy as np

#rostopic pub -1 "block_position" geometry_msgs/Pose -- '['.178' , '-.46', '-.57']' '['.4879', '.8709', '-.0285', '.0513']'
#rostopic pub -1 "block_position" geometry_msgs/Pose -- '['-.17' , '-.15', '-.']' '['.4879', '.8709', '-.0285', '.0513']'

class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Time out waiting for Joint Trajectory"
                        " Action Server to connect. Start the action server"
                        " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)
        self._done = False
        
    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)
        
    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)
        
    def stop(self):
        self._client.cancel_goal()
        
    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
    
    def result(self):
        return self._client.get_result()
    
    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

    def set_pos_callback(self, data):
        self._euclidean_goal = data
        self.execute_move(data)
        rospy.loginfo(data.position)
        rospy.loginfo(data.orientation)
    
    def execute_move(self, pos):
        rospy.loginfo('moving')
#        # block 1
        positions = {'right': [.318, 1.966, .594, -.855, -.47, .52, -2.64]}
        # dropoff
#        positions = {'right': [.306, 1.52, -.3, -1.01, -.49, .58, -2.7]}
#        positions = {'right': [0,0,0,0,0,0,0]}
#        # block 2
#        positions = {'right': [.286, 1.43, .460, -.600, -.279, .707, -3.01]}
#        limb_interface = baxter_interface.limb.Limb('right')
#        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
#        robot = URDF.from_parameter_server()
#        base_link = robot.get_root()
#        kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')
#        pose = turn_from_quat
#        q_ik = kdl_kin.inverse(pose, q+0.3)
#        self.add_point(q_ik, 0.0)
        # Read in pose data
        q = [pos.orientation.w, pos.orientation.x, pos.orientation.y, pos.orientation.z]
        p =[[pos.position.x],[pos.position.y],[pos.position.z]]
        # Convert quaternion data to rotation matrix
        R = quat_to_so3(q);
        # Form transformation matrix
        robot = URDF.from_parameter_server()
        base_link = robot.get_root()
        kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')
        q0 = kdl_kin.random_joint_angles()
        pose = kdl_kin.forward(q0)
        pose[0:3,0:3] = R
        pose[0:3,3] = p
        # Solve for joint angles
        q_ik = kdl_kin.inverse(pose, q0+0.3)
        print q_ik
        self.add_point(np.array(q_ik).tolist(), 7.0)
        self.start()
        self.wait(2)
        self._done = True
        print('Done')
    
    def ik(self, position):
        left_kin = baxter_kinematics('left')
        
        
def main():
    rospy.init_node('move_trajectory')
    traj = Trajectory('right')
    rospy.Subscriber("block_position", Pose, traj.set_pos_callback)
#    rs = baxter_interface.RobotEnable(CHECK_VERSION)
#    rs.enable()
    rospy.on_shutdown(traj.stop)
    rospy.loginfo('In loop')
    rospy.spin()   
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        
