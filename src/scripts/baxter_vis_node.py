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
state = 1
    
def xdisplay_pub(data):
    #print "entered"
    img_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    img_pub.publish(data)
    rospy.sleep(0.25)


def set_state_callback(data):
    global state
    state = data.data


def main_func():
    #Initialise Node and ROS variables
    rospy.init_node('tag_select', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    listener = tf.TransformListener()
    #rtime = rospy.Time(0)   

    #Initialize variables and empty lists
    trans = []
    rot = []
    dist = []
    id_list = []
    #tag_pose = Pose()
    tag_found = False
    global state
    state = 1


    ###Define Subsribers and Publishers
    #set callback for state

    rospy.Subscriber('state',Int16,set_state_callback)
    rospy.Subscriber("/cameras/right_hand_camera/image", Image,xdisplay_pub)
    state_pub = rospy.Publisher('state', Int16, queue_size=10, latch=True)
    tag_pub = rospy.Publisher('block_position', Pose, queue_size = 10, latch=True)

    #Sleep Before entering loop
    rospy.sleep(.5)

    '''
        try:
            (transWC,rotWC) = listener.lookupTransform('base', 'right_hand_camera', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    '''
    ####Main ROS loop to run node until shutdown
    while not rospy.is_shutdown():

        #IF statement to print current state
        if state ==1:
            print "Entered State :", state
            print "Is tag_found?:", tag_found


        ###Main State Machine Loop - Only run while state_callback is 1            
        while state == 1:# and tag_found == False:  

            rtime = rospy.Time.now()
            trans = []
            rot = []
            dist = []
            id_list = []
            
            tag_found = False
            
            rate.sleep()
            print state

            ##For loop to search through tf frames of markers
            for n in range(1,7):
                marker_id = 'ar_marker_%d'%n
                #tag_found = listener.frameExists(marker_id)
                #print "Marker found is:", 'ar_marker_%d'%n
                try:
                    rtime1 = listener.getLatestCommonTime('right_hand_camera', marker_id)
                    tag_found = True
                except (tf.Exception):
                    continue
                    
##                if rtime1 > rtime:
##                    tag_found = listener.canTransform('right_hand_camera', marker_id, rtime1)
                    
                print "Marker list: AR_Marker", n, "exist:", tag_found
                ##when a frame is found for a particular Marker "n" add to list
                ##only 1 addition per "n" in for
                if tag_found == True: 
                    #print "looking at tag:",n
                    #(transWC,rotWC) = listener.lookupTransform('base', 'right_hand_camera', rtime)
                    posCB,quatCB = listener.lookupTransform('right_hand_camera', marker_id, rospy.Time(0))
                    #print "posCB:", posCB
                    #print "quatCB", quatCB
                    trans.append(posCB)
                    rot.append(quatCB)
                    dist.append(np.linalg.norm(posCB))
                    id_list.append(n)
                    #print marker_id, ":", posCB
                    tag_found = False

            ##IF Tag list is not empty we have a tag
            if len(dist)>0: 
                #set tag_found   
                tag_found = True  

                #choose the closest tag found (if more than one)
                tag_index = np.argmin(dist)
                tag_selected = id_list[tag_index]
                tag_transform =  (trans[tag_index],rot[tag_index])
                print "tag index is", tag_index

                #get tag and world frames  into correct variables                
                posCB, quatCB = tag_transform
                posWC,quatWC = listener.lookupTransform('base', 'right_hand_camera', rtime)
                print "posCB from tf:", posCB

                #create proper format for pos and quat variables for functions
                posCB = np.array([posCB[0], posCB[1], posCB[2]])
                posWC = np.array([posWC[0], posWC[1], posWC[2]])
                quatCB = np.array([quatCB[3],quatCB[0],quatCB[1],quatCB[2]])
                quatWC = np.array([quatWC[3],quatWC[0],quatWC[1],quatWC[2]])

                #convert quat to Rotations
                rotWC = quat_to_so3(quatWC)
                rotCB = quat_to_so3(quatCB)

                #Build Transform SE3 matrix
                gWC = RpToTrans(rotWC,posWC)
                gCB = RpToTrans(rotCB,posCB)
                gWB = gWC.dot(gCB)

                #Reorient Orientation so that gripper is correct
                beta = pi
                gpick = gWB.dot(np.array([[cos(beta),0, -sin(beta),0],[0,1,0,0],[sin(beta),0, cos(beta),0],[0,0,0,1]]))
                #Recreate quatWB and rotWB (Block in world frame) from pickup Transform
                rotWB, posWB = TransToRp(gpick)
                quatWB = so3_to_quat(rotWB)

                #Create poses and orientations for block in world
                bpos = Pose()
                bpos.position.x = posWB[0]
                bpos.position.y = posWB[1]
                bpos.position.z = -.1489 #posWB[2]
                bpos.orientation.x = .99 
                bpos.orientation.y = -.024
                bpos.orientation.z = .024 
                bpos.orientation.w = .0133 

                #print statements
                print "PosWB:", posWB
                print "Distance is:", dist[np.argmin(dist)]  

            ##PUBLISH TAG if it Exists
            if tag_found == True:
                #Set locally and publish new state
                state = 2
                state_pub.publish(2)
                rospy.loginfo(2)
                #publish tage Pose
                tag_pub.publish(bpos)
                print "tag selected", tag_selected  
                print state
                del posCB,quatCB,posWC,quatWC,posWB,quatWB
                
        ###Part of Main While
        rate.sleep()


if __name__ == '__main__':
    
    main_func()
