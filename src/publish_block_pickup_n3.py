#!/usr/bin/env python

import rospy
import roslib
import math
import rosbag
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16


def publish():

	pub = rospy.Publisher('hand_position', Pose, queue_size = 10)
        state = rospy.Publisher('state', Int16, queue_size = 10)
	rospy.init_node('publisher')
	rate = rospy.Rate(60)
	pos = Pose()

    # pickup 1
#	pos.position.x = 0.5860378497814306
#	pos.position.y = -0.2398009942128175
#	pos.position.z = -0.05
#	pos.orientation.x = 0.9998354512770873
#	pos.orientation.y = -0.01507125585682272
#	pos.orientation.z = 0.0036123955077364107
#	pos.orientation.w = 0.009427524337670811
	
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
	pos.position.x = 0.8
	pos.position.y = -0.3390045079404269
	pos.position.z = 0.2
	pos.orientation.x = 0.9982126584026627
	pos.orientation.y = -0.042582835241045204
	pos.orientation.z = 0.03406633140632769
	pos.orientation.w = -0.02444740910683879

    # pickup 3
#	pos.position.x = 0.5
#	pos.position.y = -0.2398009942128175
#	pos.position.z = -0.05
#	pos.orientation.x = 0.9998354512770873
#	pos.orientation.y = -0.01507125585682272
#	pos.orientation.z = 0.0036123955077364107
#	pos.orientation.w = 0.009427524337670811
        state.publish(3)
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
    


