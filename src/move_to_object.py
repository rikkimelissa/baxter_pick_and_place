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
    
    def execute_move(self, data):
        positions = {'right': [0, 0, 0, 0, 0, 0, 0]}
#        positions = {'right': [-0.32615787,  1.04673629,  0.03483556,  0.48432787,  0.065597  ,
#       -0.06068658, -0.1981413 ]}
#        positions = {'right': [-0.17033941, -0.15113857, -1.012373  ,  1.28620707,  1.18743866,
#        0.96558141,  2.29758662]}
#        limb_interface = baxter_interface.limb.Limb('right')
#        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        robot = URDF.from_parameter_server()
        base_link = robot.get_root()
        kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')
        pose = turn_from_quat
        q_ik = kdl_kin.inverse(pose, q+0.3)
        self.add_point(q_ik, 0.0)
        self.add_point(positions['right'], 7.0)
        self.start()
        self.wait(9)
        self._done = True
        print('Done')
    
    def ik(self, position):
        left_kin = baxter_kinematics('left')
        
        
def main():
    rospy.init_node('move_trajectory')
    traj = Trajectory('right')
    rospy.Subscriber("block_position", Pose, traj.set_pos_callback)
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
    rospy.on_shutdown(traj.stop)
    rospy.loginfo('In loop')
    rospy.spin()   
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        
