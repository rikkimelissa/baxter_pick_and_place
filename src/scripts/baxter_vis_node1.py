#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import numpy as np
import cv2
import cv2.cv as cv1
from cv_bridge import CvBridge, CvBridgeError

#http://wiki.ros.org/tf/Overview/Transformations
#http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms
#to create an offset we can use the following(sec 8.1, 8.2):
from sensor_msgs.msg import Image 
#from ar_track_alvar_msgs.msg import AlvarMarker
#from sensor_msgs.msg import JointState 
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, QuaternionStamped,Vector3Stamped
from std_msgs.msg import Header
#from quat import quat_to_so3, so3_to_quat

    
def pub_tag_pose(tag_transform,WCtransform):

    print tag_transform
    (posWB, quatWB) = tag_transform

    bpos = Pose()

    bpos.position.x = posWB[0]
    bpos.position.y = posWB[1]
    bpos.position.z = posWB[2]
    bpos.orientation.x = quatWB[1]
    bpos.orientation.y = quatWB[2]
    bpos.orientation.z = quatWB[3]
    bpos.orientation.w = quatWB[0]

    tag_pub = rospy.Publisher('block_position', Pose, queue_size = 10)
    tag_pub.publish(bpos)




def multi_marker(listener,tag_found):
        
    #Creating a list for multiple markers and their locations
    trans = []
    rot = []
    dist = []
    id_list = []
    tag_pose = Pose()
    while tag_found == False:  
        for n in range(1,7):
            marker_id = 'ar_marker_%d'%n
            marker_found = listener.frameExists(marker_id)
            if marker_found == True:
                #print "looking at tag:",n
                (trans_new,rot_new) = listener.lookupTransform('right_hand_camera', marker_id, rospy.Time(0))
                trans.append(trans_new)
                rot.append(rot_new)
                dist.append(np.linalg.norm(trans_new))
                id_list.append(n)
                print marker_id, ":", trans_new
        if len(dist)>0:     
            tag_index = np.argmin(dist)
            print "tag index is", tag_index
            tag_selected = id_list[tag_index]
            tag_transform =  (trans[tag_index],rot[tag_index])
            tag_found = True
            listener.waitForTransform('base', 'right_hand_camera', rospy.Time(0), rospy.Duration(.25))
            (transWC,rotWC) = listener.lookupTransform('base', 'right_hand_camera', rospy.Time(0))
            WCtransform = (transWC,rotWC)
            print "Distance is:", dist[np.argmin(dist)]  
        #could add a statement in here to see if block is too close to neighbors
          
    return tag_found, tag_transform, WCtransform
       
    
def main_func():
    #img = Image()
    rospy.init_node('tag_select', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    count = 0
    first_count = True
    listener = tf.TransformListener()
    trans = []
    rot = []
    dist = []
    id_list = []
    tag_pose = Pose()
    tag_found = False
    state_machine = 1
    rtime = rospy.Time(0)
    # while listener.frameExists('base') == False:

    #     try:
    #         (transWC,rotWC) = listener.lookupTransform('base', 'right_hand_camera', rospy.Time(0))
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    rospy.sleep(2)
    while not rospy.is_shutdown():

        #tag_found, tag_transform, WCtransform = multi_marker(listener,tag_found)
        while tag_found == False:  
            
            for n in range(1,7):
                marker_id = 'ar_marker_%d'%n
                marker_found = listener.frameExists(marker_id)
                if marker_found == True:
                    #print "looking at tag:",n
                    #(transWC,rotWC) = listener.lookupTransform('base', 'right_hand_camera', rtime)
                    (trans_new,rot_new) = listener.lookupTransform('right_hand_camera', marker_id, rtime)
                    trans.append(trans_new)
                    rot.append(rot_new)
                    dist.append(np.linalg.norm(trans_new))
                    id_list.append(n)
                    print marker_id, ":", trans_new
            if len(dist)>0:     
                tag_index = np.argmin(dist)
                print "tag index is", tag_index
                tag_selected = id_list[tag_index]
                tag_transform =  (trans[tag_index],rot[tag_index])
                tag_found = True
                #listener.waitForTransform('base', 'right_hand_camera', rospy.Time(0), rospy.Duration(.5))
                (transWC,rotWC) = listener.lookupTransform('base', 'right_hand_camera', rospy.Time(0))
                WCtransform = (transWC,rotWC)
                print "Distance is:", dist[np.argmin(dist)]  
        count += 1



        if tag_found == True:
            #pub_tag_pose(tag_transform, WCtransform)
            print tag_transform
            (posWB, quatWB) = tag_transform

            bpos = Pose()
            bpos.position.x = posWB[0]
            bpos.position.y = posWB[1]
            bpos.position.z = posWB[2]
            bpos.orientation.x = quatWB[1]
            bpos.orientation.y = quatWB[2]
            bpos.orientation.z = quatWB[3]
            bpos.orientation.w = quatWB[0]

            tag_pub = rospy.Publisher('block_position', Pose, queue_size = 10)
            tag_pub.publish(bpos)
            rospy.sleep(2)
            break
        rate.sleep()


if __name__ == '__main__':
    
    main_func()
