#!/usr/bin/env python

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
robot = URDF.from_parameter_server()
base_link = robot.get_root()
kdl_kin = KDLKinematics(robot, base_link, 'right_gripper_base')
q0 = kdl_kin.random_joint_angles()
#a = [-0.2756209344060512,
# 1.0460338829271274,
# 0.0011107346124772377,
# 0.4847025119189139,
# -0.07919930006668796,
# -0.0609079618195949,
# 0.027413666235402978]
for ind in range(len(q)):
    q[ind] = current_angles[ind]
    
for ind in range(len(q_ik)):
    q[ind] = current_angles[ind]
#q = q*0
pose = kdl_kin.forward(q0) # forward kinematics (returns homogeneous 4x4 numpy.mat)
pose[2,3] = 0.01
q_ik = kdl_kin.inverse(pose, q+0.3) # inverse kinematics
if q_ik is not None:
    pose_sol = kdl_kin.forward(q_ik) # should equal pose
    print pose
    print pose_sol


limb_interface = baxter_interface.limb.Limb('right')
current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
x = limb.joint_angles()

for ind, joint in enumerate(limb_interface.joint_names()):
    x[joint] = q_ik[ind]


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

# out below table
limb_interface.joint_angles()
Out[36]: 
{'right_e0': 0.07746602969970703,
 'right_e1': 1.6743400280639649,
 'right_s0': -0.17410681922607424,
 'right_s1': -0.13230584280395508,
 'right_w0': -1.1313108297729493,
 'right_w1': 0.44523792317504884,
 'right_w2': -2.741223664819336}

joints array([-0.1752573 , -0.13230584,  0.07746603,  1.67434003, -1.13131083,
        0.44523792, -2.74160716])
matrix([[ 0.91313312, -0.19469164, -0.35816626,  0.25647275],
        [-0.22851797, -0.97202839, -0.0542249 , -0.6235352 ],
        [-0.33759064,  0.13136198, -0.93208186, -0.2212066 ],
        [ 0.        ,  0.        ,  0.        ,  1.        ]])
        
limb_interface.endpoint_pose()
{'orientation': Quaternion(x=0.9768890955607737, y=-0.10760651246696384, z=-0.1783812627776889, w=0.047839926201734884),
 'position': Point(x=0.23995560309506747, y=-0.626187563531023, z=-0.26322845245944343)}
 
from geometry_msgs.msg import Pose
pos = Pose()
pos.position.x = 0.23995560309506747
pos.position.y = -0.626187563531023
pos.position.z = -0.26322845245944343 
pos.orientation.x = 0.9768890955607737
pos.orientation.y = -0.10760651246696384
pos.orientation.z = -0.1783812627776889
pos.orientation.w = 0.047839926201734884

# Read in pose data
q = [pos.orientation.w, pos.orientation.x, pos.orientation.y, pos.orientation.z]
p =[[pos.position.x],[pos.position.y],[pos.position.z]]
# Convert quaternion data to rotation matrix
R = quat_to_so3(q);
# Form transformation matrix
q0 = kdl_kin.random_joint_angles()
pose = kdl_kin.forward(q0)
pose[0:3,0:3] = R
pose[0:3,3] = p
# Solve for joint angles
q_ik = kdl_kin.inverse(pose, q0+0.3)



 # block pickup position
limb_interface.joint_angles()
Out[38]: 
{'right_e0': 0.31868450831909184,
 'right_e1': 1.966946863018799,
 'right_s0': 0.5948010498229981,
 'right_s1': -0.855194288269043,
 'right_w0': -0.47706802448730473,
 'right_w1': 0.5207864768920899,
 'right_w2': -2.6422819041137697}

limb_interface.endpoint_pose()
Out[39]: 
{'orientation': Quaternion(x=0.9998354512770873, y=-0.01507125585682272, z=0.0036123955077364107, w=0.009427524337670811),
 'position': Point(x=0.5860378497814306, y=-0.2398009942128175, z=0.014414851085098497)}
 
 

current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
for ind in range(len(q)):
    q[ind] = current_angles[ind]
#q = q*0
pose = kdl_kin.forward(q)
pose
Out[44]: 
matrix([[ 0.99953372, -0.03002665,  0.00554463,  0.58510058],
        [-0.02991357, -0.99936304, -0.01946025, -0.23959431],
        [ 0.00612542,  0.01928532, -0.99979526,  0.05891605],
        [ 0.        ,  0.        ,  0.        ,  1.        ]])


# block dropoff position
{'right_e0': 0.3064126620300293,
 'right_e1': 1.5286118533813478,
 'right_s0': -0.31523305155029296,
 'right_s1': -1.019330232385254,
 'right_w0': -0.4912573467590332,
 'right_w1': 0.5844466795166016,
 'right_w2': -2.789927554779053}
 {'orientation': Quaternion(x=0.8688237867733386, y=-0.4151335977354022, z=0.21507338622355585, w=0.16295018289781277),
 'position': Point(x=0.5297221887963302, y=-0.887243533851046, z=0.24499675859910147)}
 

#2nd block pose
{'right_e0': 0.28647091181030276,
 'right_e1': 1.4319710638549805,
 'right_s0': 0.46019423583984376,
 'right_s1': -0.6005534777709961,
 'right_w0': -0.2795679982727051,
 'right_w1': 0.7071651424072266,
 'right_w2': -3.0173402063232424}
{'orientation': Quaternion(x=0.9982126584026627, y=-0.042582835241045204, z=0.0340663314063276, w=-0.02444740910683879),
 'position': Point(x=0.7340259833173312, y=-0.3390045079404269, z=-0.008877793884743197)}



'''
Things to work on:
- Make it slower
- Make it smarter

rostopic pub -1 "state" std_msgs/Int16 -- 1

- creating trajectories and moving with set_joint_velocities
- using position mode with JTAS
- using velocity mode with JTAS
- using inverse dynamics with JTAS






