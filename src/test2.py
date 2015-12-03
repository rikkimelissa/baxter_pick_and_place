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
        
#                # Compute straight-line trajectory for path
#        N = 5
#        Xlist = CartesianTrajectory(Xstart, Xend, 1, N, 5)
#        thList = np.empty((N,7))
#        thList[0] = q0;
#        
#        for i in range(N-1):
#        # Solve for joint angles
#            seed = 0
#            q_ik = kdl_kin.inverse(Xlist[i+1], thList[i])
#            while q_ik == None:
#                seed += 0.3
#                q_ik = kdl_kin.inverse(pose, q0+seed)
#            thList[i+1] = q_ik
#            rospy.loginfo(q_ik)
        
#        # Solve for joint angles
        seed = 0.3
        q_ik = kdl_kin.inverse(pose, q0+seed)
        while q_ik == None:
            seed += 0.3
            q_ik = kdl_kin.inverse(pose, q0+seed)
        rospy.loginfo(q_ik)
        
#        q_list = JointTrajectory(q0,q_ik,1,100,5)
        
#        for q in q_list:
            # Format joint angles as limb joint angle assignment      
        angles = limb_interface.joint_angles()
        for ind, joint in enumerate(limb_interface.joint_names()):
            angles[joint] = q_ik[ind]
#            rospy.loginfo(angles)
        rospy.sleep(.003)
        
        # Send joint move command
#            limb_interface.set_joint_position_speed(.01)
        limb_interface.set_joint_position_speed(.3)
        limb_interface.move_to_joint_positions(angles)
