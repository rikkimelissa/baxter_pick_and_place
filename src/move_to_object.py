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

def main():

    rospy.init_node('move_trajectory')
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
#    positions = {'right': [-0.11, -.62, -1.15, 1.32, .80, 1.27, 2.39]}
    positions = {'right': [0, 0, 0, 0, 0, 0, 0]}
    traj = Trajectory('right')
    rospy.on_shutdown(traj.stop)
    limb_interface = baxter_interface.limb.Limb('right')
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    traj.add_point(current_angles, 0.0)
    traj.add_point(positions['right'], 7.0)
    traj.start()
    traj.wait(9)
    print("Done")
    
    
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        
