#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion

import tf2_ros
import tf2_msgs.msg

###import my message
from tmc_vision_msgs.msg import DetectionArray , Detection, Label
from tmc_vision_msgs.msg import yolo_store_msg_Array, yolo_store_msg
from std_msgs.msg import Header , String


topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray,queue_size=1)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    
def visualize():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()


# rospy.init_node('register')

markerArray = MarkerArray()

count = 0.0
cnt = 0.0
MARKERS_MAX = 30
id = 0

if __name__ == '__main__':
    rospy.init_node('visualize', anonymous=True)
    topic = 'visualization_marker_array'
    publisher = rospy.Publisher(topic, MarkerArray,queue_size=1)
    r = rospy.Rate(100)
    r_max_id = rospy.Rate(0.5)
    msg_store = MessageStoreProxy()
    while ok():
        print("visualizing message  {} ",.format(str(id))
        mongo_array = msg_store.query(yolo_store_msg_Array._type)#  sort_query = [], projection_query = {}, limit=0)
        if id <= mongo_array.__len__():
            current_array = mongo_array[id]
            for i in current_array:
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.type = marker.CUBE
                marker.action = marker.ADD
                s=0.3
                marker.scale.x = current_array[i].yolo_store_msg
                marker.scale.y = current_array[i].yolo_store_msg
                marker.scale.z = current_array[i].yolo_store_msg
                marker.color.a = 0.6
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z 
                markerArray.markers.append(marker)
                for m in markerArray.markers:
                    m.id = id
                    id += 1
            publisher.publish(markerArray)
            r.sleep()

        else: r_max_id.sleep()

#    # Publish the MarkerArray
#    publisher.publish(markerArray)

##    # marker from it when necessary
#    if(cnt > (MARKERS_MAX)):
#        markerArray.markers.pop(0)

#    

#    # Renumber the marker IDs
#    id = 0


#    count += 1.0/div
#    cnt += 1.0/div

