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
from ar_track_alvar_msgs.msg import AlvarMarker
#from sensor_msgs.msg import JointState 
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, QuaternionStamped,Vector3Stamped
from std_msgs.msg import Header
#from quat import quat_to_so3, so3_to_quat


def xdisplay_pub(data,count):
    print "entered"
    if count%20 ==0:
        #img_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=100)
        #img_pub.publish(data)
        print "image publishing"

def starting_arm_position():
    #enter something here 
    x =1
    
def circle_find(img_msg,count):

    bridge = CvBridge()
    if count%30 == 0:

        try:
           frame1 = bridge.imgmsg_to_cv2(img_msg, "bgr8")
           #cv2.VideoCapture()
           cv_image = frame1
           cv2.imwrite(cv_image.png,cv_image)
        except CvBridgeError, e:
            print("==[CAMERA MANAGER]==", e)
        #red = np.uint([[[255,0,0]]])

        #hsv_red = cv2.cvtColor(red,cv2.COLOR_BGR2HSV)
        #print hsv_red
        print cv_image.shape            
        img = cv2.medianBlur(cv_image,5)

        
        gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        circles = cv2.HoughCircles(gray_img,cv1.CV_HOUGH_GRADIENT,1,20,
                                    param1=50,param2=30,minRadius=1,maxRadius=20)

        #circles = np.uint16(np.around(circles))
        '''
        if len(circles)>0:
            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(gray_img,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(gray_img,(i[0],i[1]),2,(0,0,255),3)

        cv2.imshow('detected circles',gray_img)

        #convert open cv image to ROS image
        '''
        im_m = bridge.cv2_to_imgmsg(gray_img, "mono8")
        pub_img = rospy.Publisher('image_topic',Image, queue_size = 10)
        pub_img.publish(im_m)

        '''
        cv2.waitKey(15)

        cv2.destroyAllWindows()
        '''
    
    
def pub_tag_pose(tag_transform,WCtransform):
    # you enter here posCB and quatCB which is pose data in quaternions
    #listener = tf.TransformListener()
    #marker_id = 'ar_marker_%d'%tag_id
    #print "pub_tag_pose: markerid", marker_id
    #marker_found = listener.frameExists('ar_marker_%d'%tag_id)
    #print marker_found
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
    '''
    quatWC = [quatWC[1:4], quatWC[0]]
    quatCB = [quatCB[1:4], quatCB[0]]

    rotWC = quat_to_so3(quatWC)
    rotCB = quat_to_so3(quatCB)
    gWC = RpToTrans(rotWC,posWC)
    gCB = RpToTrans(rotCB,posCB)
    gWB = gWC.dot(gCB)
    rotWB, posWB = TransToRp(gWB)
    quatWB = so3_to_quat(rotWB)
    '''



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
    tag_found = False
    state_machine = 1
    while not rospy.is_shutdown():

        #if count%5 == 0
        #    rospy.Subscriber("/cameras/right_hand_camera/image", Image, get_tag_pose)

        #This puts the image onto Baxter's head every 2 seconds by calling the xdisplay_pub topic
        #rospy.Subscriber("/usb_cam/image_raw", Image,xdisplay_pub,count)
        #rospy.Subscriber("/cameras/right_hand_camera/image", Image,xdisplay_pub,count)

        ###This piece is for finding circles###
        ###not finished yet, i only want to save one picture every second or two and use that###
        #rospy.Subscriber("/usb_cam/image_raw", Image, circle_find,count)
        #rospy.Subscriber("/usb_cam/image_raw", Image, circle_find,count)

        ###This piece finds the closest marker when multiple unique markers are used###
        #rospy.Subscriber("/usb_cam/image_raw", Image, multi_marker,listener,tag_found)
        tag_found, tag_transform, WCtransform = multi_marker(listener,tag_found)
        #pub_tag_pose(listener,tag_selected)        


        count += 1
        #print count
        if tag_found == True:
            #print "tag selected", tag_selected
            pub_tag_pose(tag_transform, WCtransform)
            rospy.sleep(2)
            break
        rate.sleep()


if __name__ == '__main__':
    
    main_func()
