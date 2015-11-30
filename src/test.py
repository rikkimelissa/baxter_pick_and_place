#!/usr/bin/env python

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
robot = URDF.from_parameter_server()
base_link = robot.get_root()
kin = KDLKinematics(robot, base_link, 'right_gripper_base')
kdl_kin = KDLKinematics(robot, base_link, end_link)
q = kdl_kin.random_joint_angles()
#q = q*0
pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)
#pose[0,3] = .05
q_ik = kdl_kin.inverse(pose, q+0.3) # inverse kinematics
if q_ik is not None:
    pose_sol = kdl_kin.forward(q_ik) # should equal pose
    print pose
    print pose_sol


#import rospy
#from baxter_pykdl import baxter_kinematics

#kin = baxter_kinematics('right')

#pos = [0.582583, -0.180819, 0.216003]
#rot = [0.03085, 0.9945, 0.0561, 0.0829]
#b = kin.inverse_kinematics(pos,rot)
#print b

#a = kin.forward_position_kinematics()
#print a
#a[0:3] = a[0:3] + .05
#pos = [a[0], a[1], a[2]]
#rot = [a[3], a[4], a[5], a[6]]
#b = kin.inverse_kinematics(pos,rot)
##print b
#cur_type_values = kin._limb_interface.joint_angles()
#for idx, name in enumerate(cur_type_values):
#    cur_type_values[name] = b[idx]
#a2 = kin.forward_position_kinematics(cur_type_values)
#print a2

#c = [b-b2 for b,b2 in zip(a,a2)]
#print sum(c)





