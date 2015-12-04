#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import PoseStamped,Pose

pose_msg = Pose()

count = 1

def listener():
   
    rospy.init_node('block_pose')
    print "Node for block pose"
    while not rospy.is_shutdown(): 
		x=rospy.Subscriber('/ar_pose_marker', AlvarMarker ,callback)
		#print "success"
		#publisher()
		#print count
		global count
		count = count + 1
		print x
		

def callback(data):
	print "callback"
 	print data
	# pose_msg.position.x = data.pose.pose.position.x
	# pose_msg.position.y = data.pose.pose.position.y
	# pose_msg.position.z = data.pose.pose.position.z
	# pose_msg.orientation.x = data.pose.pose.orientation.x
	# pose_msg.orientation.y = data.pose.pose.orientation.y
	# pose_msg.orientation.z = data.pose.pose.orientation.z
	# pose_msg.orientation.w = data.pose.pose.orientation.w
	return 3





# def publisher():
#     pub_pose = rospy.Publisher('block_position', Pose, queue_size=10)
    
#     #rospy.init_node('input', anonymous=True)
#     rate = rospy.Rate(10)
   
#     while not rospy.is_shutdown():
       
        
       
#         pub_pose.publish(pose_msg)
        
#         print "pose_msg: "
#         print pose_msg
#         rate.sleep()
if __name__ == "__main__":
    listener()
    
