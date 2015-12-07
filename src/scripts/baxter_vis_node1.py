#!/usr/bin/env python  
import roslib
import rospy
from math import pi, cos, sin
import tf
import numpy as np
#import cv2
#import cv2.cv as cv1
#from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image 
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, QuaternionStamped,Vector3Stamped
from std_msgs.msg import Header, Int16
from quat import quat_to_so3, so3_to_quat
from functions import RpToTrans, TransToRp

    
def xdisplay_pub(data,count):
    #print "entered"
    img_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=100)
    img_pub.publish(data)
    rospy.sleep(0.25)


def set_state_callback(data):
    state = data.data


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
    state = 1
    rtime = rospy.Time(0)
    rospy.Subscriber('state',Int16,set_state_callback)
    # while listener.frameExists('base') == False:

    #     try:
    #         (transWC,rotWC) = listener.lookupTransform('base', 'right_hand_camera', rospy.Time(0))
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    rospy.sleep(1)


    while not rospy.is_shutdown():
        #if count%10==0:
        rospy.Subscriber("/cameras/right_hand_camera/image", Image,xdisplay_pub,count)

        #tag_found, tag_transform, WCtransform = multi_marker(listener,tag_found)
        while tag_found == False:  
            #posCB, quatCB, posWC, quatWC = multi_marker(listener)
            for n in range(1,7):
                marker_id = 'ar_marker_%d'%n
                marker_found = listener.frameExists(marker_id)
                if marker_found == True:
                    #print "looking at tag:",n
                    #(transWC,rotWC) = listener.lookupTransform('base', 'right_hand_camera', rtime)
                    posCB,quatCB = listener.lookupTransform('right_hand_camera', marker_id, rtime)
                    print "posCB:", posCB
                    print "quatCB", quatCB
                    trans.append(posCB)
                    rot.append(quatCB)
                    dist.append(np.linalg.norm(posCB))
                    id_list.append(n)
                    print marker_id, ":", posCB
            if len(dist)>0:     
                tag_index = np.argmin(dist)
                print "tag index is", tag_index
                tag_selected = id_list[tag_index]
                tag_transform =  (trans[tag_index],rot[tag_index])
                tag_found = True
                #listener.waitForTransform('base', 'right_hand_camera', rospy.Time(0), rospy.Duration(.5))
                
                posCB, quatCB = tag_transform
                posWC,quatWC = listener.lookupTransform('base', 'right_hand_camera', rospy.Time(0))

                print "posCB after if:", posCB
                print "quatCB acter if:", quatCB

                print "posWC", posWC
                print "quatWC", quatWC



                posCB = np.array([posCB[0], posCB[1], posCB[2]])
                posWC = np.array([posWC[0], posWC[1], posWC[2]])
                quatCB = np.array([quatCB[3],quatCB[0],quatCB[1],quatCB[2]])
                quatWC = np.array([quatWC[3],quatWC[0],quatWC[1],quatWC[2]])
                

                # print tag_transform
         
                print "Distance is:", dist[np.argmin(dist)]  

        count += 1



        #if tag_found == True:
        #pub_tag_pose(tag_transform, WCtransform)
        #print tag_transform
       
        rotWC = quat_to_so3(quatWC)
        rotCB = quat_to_so3(quatCB)
        gWC = RpToTrans(rotWC,posWC)
        print "gWC is: ", gWC
        gCB = RpToTrans(rotCB,posCB)
        gWB = gWC.dot(gCB)
        beta = pi
        gpick = gWB.dot(np.array([[cos(beta),0, -sin(beta),0],[0,1,0,0],[sin(beta),0, cos(beta),0],[0,0,0,1]]))
        print "gWB is: ", gWB
        print "gCB is: ", gCB
        rotWB, posWB = TransToRp(gpick)
        quatWB = so3_to_quat(rotWB)

        print "quatWB", quatWB
        print "PosWB:", posWB

        bpos = Pose()
        bpos.position.x = posWB[0]
        bpos.position.y = posWB[1]
        bpos.position.z = posWB[2]
        bpos.orientation.x = quatWB[1]
        bpos.orientation.y = quatWB[2]
        bpos.orientation.z = quatWB[3]
        bpos.orientation.w = quatWB[0]
        tag_found = False

        state_pub = rospy.Publisher('state', Int16, queue_size=10)
        tag_pub = rospy.Publisher('block_position', Pose, queue_size = 10)

        for i in range(10):
            # state_pub = rospy.Publisher('state', Int16, queue_size=10)
            state_pub.publish(2)

            # tag_pub = rospy.Publisher('block_position', Pose, queue_size = 10)
            tag_pub.publish(bpos)

            print "tag selected", tag_selected  
        #rospy.sleep(1)
            #break
        print state
        rate.sleep()


if __name__ == '__main__':
    
    main_func()
