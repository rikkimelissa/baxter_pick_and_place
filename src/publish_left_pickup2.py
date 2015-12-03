#!/usr/bin/env python

import rospy
import roslib
import math
import rosbag
from geometry_msgs.msg import Pose


def publish():

	pub = rospy.Publisher('block_position', Pose, queue_size = 10)
	rospy.init_node('publisher')
	rate = rospy.Rate(60)
	pos = Pose()

    # pickup 1
#	pos.position.x = 0.6764680992739975
#	pos.position.y = 0.25317487471426436
#	pos.position.z = -0.03943550506965804
#	pos.orientation.x = -0.02499840445577016
#	pos.orientation.y = 0.9990242278969153
#	pos.orientation.z = 0.009559997873184244
#	pos.orientation.w = 0.03513229696878465
	
	# dropoff
#	pos.position.x = 0.5297221887963302
#	pos.position.y = -0.887243533851046
#	pos.position.z = 0.24499675859910147
#	pos.orientation.x = 0.8688237867733386
#	pos.orientation.y = -0.4151335977354022
#	pos.orientation.z = 0.21507338622355585
#	pos.orientation.w = 0.16295018289781277
#	
#    # pickup 2
	pos.position.x = 0.6764680992739975
	pos.position.y = 0.25317487471426436
	pos.position.z = -0.03943550506965804
	pos.orientation.x = -0.02499840445577016
	pos.orientation.y = 0.9990242278969153
	pos.orientation.z = 0.009559997873184244
	pos.orientation.w = 0.03513229696878465
	rospy.loginfo(pos)
	pub.publish(pos)
#	drive_cmd.position = [.178 , -.46, -.57]
#	drive_cmd.orientation = [1,2,3,4]
	
#	while not rospy.is_shutdown():
#    rospy.loginfo(drive_cmd)
#    pub.publish(drive_cmd)
                                                                                                                         
if __name__ == "__main__":
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
    


