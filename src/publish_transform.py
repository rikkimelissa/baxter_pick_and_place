
# you enter here posCB and quatCB which is pose data in quaternions
q = [pos.orientation.w, pos.orientation.x, pos.orientation.y, pos.orientation.z]
p =[[pos.position.x],[pos.position.y],[pos.position.z]]
        
import tf
from quat import quat_to_so3, so3_to_quat
from functions import RpToTrans, TransToRp
from geometry_msgs.msg import Pose
from ar_track_alvar_msgs.msg import AlvarMarker

rospy.Subscriber("/ar_pose_marker", AlvarMarker,data_callback)
def data_callback(data):  
    
listener = tf.TransformListener()
posWC, quatWC = listener.lookupTransform('/base','/right_hand_camera',rospy.Time(0))
posWC = np.array([.706, .124, .306])
quatWC = [-.007, .763, -.645, -.032]
posCB = np.array([.031, -.106, .450]) 
quatCB = [-.03, .0234, .999, -.014]
quatWC = np.array([quatWC[0], quatWC[1], quatWC[2], quatWC[3]])
quatCB = np.array([quatCB[0], quatCB[1], quatCB[2], quatCB[3]])
rotWC = quat_to_so3(quatWC)
rotCB = quat_to_so3(quatCB)
gWC = RpToTrans(rotWC,posWC)
gCB = RpToTrans(rotCB,posCB)
gWB = gWC.dot(gCB)

rotWB, posWB = TransToRp(gWB)
quatWB = so3_to_quat(rotWB)

pub = rospy.Publisher('block_position', Pose, queue_size = 10)
pos = Pose()
pos.position.x = posWB[0]
pos.position.y = posWB[1]
pos.position.z = posWB[2]
pos.orientation.x = rotWB[1]
pos.orientation.y = rotWB[2]
pos.orientation.z = rotWB[3]
pos.orientation.w = rotWB[0]










