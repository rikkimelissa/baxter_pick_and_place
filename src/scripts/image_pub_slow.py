#!/usr/bin/env python 

#import cv2
#import cv_bridge
import rospy
from sensor_msgs.msg import Image


def xdisplay_pub(data):
    pub = rospy.Publisher('/slow_image', Image, queue_size=10)
    pub.publish(data)


    
def sub_image():
    rospy.init_node('image_slow', anonymous=False)
    #rate = rospy.Rate(1) # 1 hz
    count = 0
    #may add some comment about timing
    while not rospy.is_shutdown():
        rospy.Subscriber("/usb_cam/image_raw", Image, xdisplay_pub)
        #replace subscibed topic to /cameras/head_camera
        # Sleep to allow for image to be published.
        #this should allow this cycle to sleep for 0.25 seconds per loop
        print count
        count = count + 1
        rospy.sleep(0.25)

    
if __name__ == '__main__':
    sub_image()
