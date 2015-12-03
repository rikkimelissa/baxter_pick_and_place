#!/usr/bin/env python  
import roslib
import rospy


from baxter_core_msgs.msg import HeadPanCommand
from baxter_interface.camera import CameraController
from std_msgs.msg import String

#set the head so that it is in correct location
#can a node publish multiple topics?

'''
def callback(state,HS,rate)

    while state="some state":
        pub.publish(HS)
        rate.sleep()
        #print HS
        
    #the following cmd line may be used: rostopic pub /robot/head/command_head_pan  baxter_core_msgs/HeadPanCommand -- 0.0 10

'''

def head_state():
    pub = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, queue_size=10)
    rospy.init_node('head_state_node', anonymous=False)
    HS = HeadPanCommand()
    HS.speed = 10
    HS.target = 0.0
    rate = rospy.Rate(10) # 10hz
    
    #when using the state machine use the subscriber below to intiate callback
    #and pass rate 
    #rospy.Subscriber('/state_or_whatever',....,callback, HS,rate)
    
    while not rospy.is_shutdown():
        pub.publish(HS)
        rate.sleep()
        #print HS
    


if __name__ == '__main__':
    try:
        head_state()
    except rospy.ROSInterruptException:
        pass
