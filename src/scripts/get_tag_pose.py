#!/usr/bin/env python 

#import cv2
#import cv_bridge
import rospy
import tf
#from ar_track_alvar_msgs.msg import AlvarMarker
from sensor_msgs.msg import Image
#from sensor_msgs.msg import JointState 
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

import roslib
import math

#from visualization_msgs.msg import Marker


def select_tag(data):
    pub = rospy.Publisher('new_pose', Pose, latch=True, queue_size=10)
    in_pose = Pose()
    pub.publish(data)


    
def get_pose():
    rospy.init_node('get_pose', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    listener = tf.TransformListener()
    #to create an offset we can use the following(sec 8.1, 8.2):
    #http://wiki.ros.org/tf/Overview/Transformations
    #transform formatting:
    #http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms
    #initialize lists
    transe = []
    rot = []
   
    while not rospy.is_shutdown(): #while != some function that contniues to look until the state changes
        #rospy.Subscriber("/ar_pose_marker", av_msg, callback) 
        #av_msg = AlvarMarker()
        #tag_id = av_msg.id

        #this method 
        
        try:
            (trans[0],rot[0]) = listener.lookupTransform('/head_camera', '/ar_marker_1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            (trans[1],rot[1]) = listener.lookupTransform('/head_camera', '/ar_marker_2', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            (trans[2],rot[2]) = listener.lookupTransform('/head_camera', '/ar_marker_3', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            (trans[3],rot[3]) = listener.lookupTransform('/head_camera', '/ar_marker_4', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            (trans[4],rot[4]) = listener.lookupTransform('/head_camera', '/ar_marker_5', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            (trans[5],rot[5]) = listener.lookupTransform('/head_camera', '/ar_marker_6', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
        for i,j in trans,rot:
            #run until both lists are full? or until some length paramet is satisfied?
            
        #call select tag function
        select_tag(trans,rot)
        rate.sleep()

    
if __name__ == '__main__':
    get_pose()
    
    #try:
        #publisher()
    #except rospy.ROSInterruptException:
        #pass
