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


#two goals:
#1. create algorithm to sort if many blocks with same tage are seen, or if many blocks with different tags are seen
#2. select the block to get
#3. cutout the number of images broadcast to only get a picture per second


def select_tag(data):
    pub = rospy.Publisher('new_pose', Pose, latch=True, queue_size=10)
    in_pose = Pose()
    pub.publish(data)


    
def get_pose():
    rospy.init_node('get_pose', anonymous=False)
    subrate = rospy.Rate(1) # 10hz
    listener = tf.TransformListener()
    
    #to create an offset we can use the following(sec 8.1, 8.2):
    #http://wiki.ros.org/tf/Overview/Transformations
    #transform formatting:
    #http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms
    #initialize lists
    trans = []
    rot = []
   
    while not rospy.is_shutdown(): #while != some function that contniues to look until the state changes

        #using time http://wiki.ros.org/rospy/Overview/Time
        now_time =rospy.Time.now()
        #create a time of 10 seconds = time_10 = rospy.Time(10)
        

        dt = 0
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
            
        for i in trans,rot:
            #run until both lists are full? or until some length paramet is satisfied?
            
        #call select tag function
        select_tag(trans,rot)
        subrate.sleep()

    
if __name__ == '__main__':
    get_pose()
    
    #try:
        #publisher()
    #except rospy.ROSInterruptException:
        #pass
