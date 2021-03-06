#!/usr/bin/env python 

#import cv2
#import cv_bridge
import rospy
from sensor_msgs.msg import Image


def xdisplay_pub(data):
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)
    #pub = rospy.Publisher('/robot_image', Image, latch=True, queue_size=10)
    pub.publish(data)

    
def get_image():
    #img = Image()
    rospy.init_node('image_data', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    #count = 0
    #rostopic pub /robot/head/command_head_pan baxter_core_msgs/HeadPanCommand -- 0 6

    while not rospy.is_shutdown():
        #rospy.Subscriber("/usb_cam/image_raw", Image, xdisplay_pub)
        rospy.Subscriber("/cameras/right_hand_camera/image", Image, xdisplay_pub)
        #print count
        #count = count + 1
        rate.sleep()
       
'''
    #may add some comment about timing
    while not rospy.is_shutdown():
        #rospy.Subscriber("/usb_cam/image_raw", Image, xdisplay_pub)
        rospy.Subscriber("/slow_image", Image, xdisplay_pub)
        #replace subscibed topic to /cameras/head_camera
        # Sleep to allow for image to be published.
        rate.sleep()
'''
if __name__ == '__main__':
    get_image()
