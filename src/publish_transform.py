
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
posWC, quatWC = listener.lookupTransform('/base','/head_camera',rospy.Time(0))
quatWC = [quatWC[1:4], quatWC[0]]
quatCB = [quatCB[1:4], quatCB[0]]
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










