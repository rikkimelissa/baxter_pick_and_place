#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import numpy as np
#http://wiki.ros.org/tf/Overview/Transformations
#http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms
#to create an offset we can use the following(sec 8.1, 8.2):
from sensor_msgs.msg import Image 
from ar_track_alvar_msgs.msg import AlvarMarker
#from sensor_msgs.msg import JointState 
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, QuaternionStamped,Vector3Stamped
from std_msgs.msg import Header


def xdisplay_pub(data):
    img_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=100)
    #pub = rospy.Publisher('/robot_image', Image, latch=True, queue_size=10)
    img_pub.publish(data)
    
def select_tag(data):
    tag_pub = rospy.Publisher('new_pose', Pose, latch=True, queue_size=10)
    in_pose = Pose()
    tag_pub.publish(data)

    
def get_tag_pose():

    listener = tf.TransformListener()
    trans1 = 0
    
    print listener.frameExists('ar_marker_4')
    print listener.frameExists('head_camera')
    if listener.frameExists('ar_marker_4') == True and listener.frameExists('head_camera') ==True:
        (trans1,rot1) = listener.lookupTransform('ar_marker_4', 'ar_marker_5', rospy.Time(0))
        print trans1
    
    #call select tag function

def multi_marker(trans,rot,dist,id_list,listener):
        
    #Creating a list for multiple markers and their locations
    trans = []
    rot = []
    dist = []
    id_list = []
    print len(dist)
    while len(dist)<1:  
        for n in range(1,7):
            marker_id = 'ar_marker_%d'%n
            marker_found = listener.frameExists(marker_id)
            if marker_found == True:
                print "looking at tag:",n
                (trans_new,rot_new) = listener.lookupTransform('head_camera', marker_id, rospy.Time(0))
                trans.append(trans_new)
                rot.append(rot_new)
                dist.append(np.linalg.norm(trans_new))
                id_list.append(n)
        if len(dist)>0:     
            tag_index = np.argmin(dist)
            print "tag index is", tag_index
            tag_selected = id_list[tag_index]
            tag_transform =  (trans[tag_index],rot[tag_index])
        #could add a statement in here to see if block is too close to neighbors
        
    print "Distance is:", dist[np.argmin(dist)]
    print "Tag Selected is:",tag_selected
    print "Transform Selected is:", tag_transform
    
    return tag_selected, tag_transform
       
    
def main_func():
    #img = Image()
    rospy.init_node('tag_select', anonymous=False)
    rate = rospy.Rate(1) # 10hz
    first_count = True
    listener = tf.TransformListener()
    trans = []
    rot = []
    dist = []
    id_list = []
    while not rospy.is_shutdown():
        #if count%10 == 0:
            #rospy.Subscriber("/usb_cam/image_raw", Image, xdisplay_pub)
            #rospy.Subscriber("/cameras/right_hand_camera/image", Image, xdisplay_pub)
        #if count%5 == 0
        #    rospy.Subscriber("/cameras/right_hand_camera/image", Image, get_tag_pose)
        '''
        list_frames = listener.getFrameStrings()
        marker_found = listener.frameExists('ar_marker_5')
        if marker_found == True:
            (trans_new,rot_new) = listener.lookupTransform('head_camera', 'ar_marker_5', rospy.Time(0))
            #print trans_new
        
        if marker_found == True and first_count == True:
            first_count = False 
            (trans_old,rot_old) = (trans_new,rot_new)
            trans.append(trans_new)
            rot.append(rot_new)
            print "Marker Found2"
            print trans
        
        if marker_found == True and first_count == False:
            for i in range(len(trans)):
            a = np.round(trans_old[0],2).tolist()
            b = np.round(trans_new[0],2).tolist()
            print "New Marker??"
            
            if a != b:
                print "NEW MARKER FOUND"
                (trans_old,rot_old) = (trans_new, rot_new)
                trans.append(trans_new)
                rot.append(rot_new)
        '''

        '''
        try:
           (trans_new,rot_new) = listener.lookupTransform('head_camera', 'ar_marker_5', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
           marker_found = False
           #print "Marker Found?:", marker_found
        else: 
           #print trans
           x =1
        '''
        #if we want to use multiple markers and pick them up based on distance
        tag_selected, tag_transform = multi_marker(trans,rot,dist,id_list, listener)
        rate.sleep()

if __name__ == '__main__':
    
    main_func()
