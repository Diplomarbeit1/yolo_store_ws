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
from std_msgs.msg import Int32

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


# topic = 'visualization_marker_array'
# publisher = rospy.Publisher(topic, MarkerArray,queue_size=1)

def visualize():
    rospy.init_node('listener', anonymous=True)
    rospy.spin()


# rospy.init_node('register')

markerArray = MarkerArray()
markerArray_text = MarkerArray()
count = 0.0
cnt = 0.0
#MARKERS_MAX = 1000
id = 0


if __name__ == '__main__':
    rospy.init_node('visualize', anonymous=True)
    topic_mark = '/yolo_store/marker_array'
    topic_text = '/yolo_store/marker_text_array'
    topic_ind = '/yolo_store/max_index'
    publisher_mark = rospy.Publisher(topic_mark, MarkerArray,queue_size=1)
    publisher_text = rospy.Publisher(topic_text, MarkerArray,queue_size=1)
    publisher_id = rospy.Publisher(topic_ind, Int32 ,queue_size=1)
    r = rospy.Rate(100)
    r_max_id = rospy.Rate(0.1)
    msg_store = MessageStoreProxy()
    while not rospy.is_shutdown():
        #print("visualizing message  {} ",.format(str(id))
	
	start =  rospy.get_rostime()
	now = rospy.get_rostime()

        mongo_array = msg_store.query(yolo_store_msg_Array._type)#  sort_query = [], projection_query = {}, limit=0)
        max_id = mongo_array.__len__()
        print("visualize msg: "+str(id)+" / "+str(max_id))
        #print("max id:",max_id)
        publisher_id.publish(max_id)

	print("query:"+str((rospy.get_rostime()-now).to_sec()))
	now = rospy.get_rostime()

        #print((mongo_array[1][0].msgs[0]))
        if id < max_id & max_id > 0:
           
            current_array = mongo_array[id][0]
            i=0
            for i in range(current_array.msgs.__len__()):
                if(i<=0): 
                    ad=0.01 
                else: 
                    ad=0
                z=1
                marker = Marker()
                marker.header.frame_id = "map"
                marker.type = marker.CUBE
                marker.action = marker.ADD
                #print(int(round(i/z))*10)
                if(current_array.msgs[int(round(i/z))*z].lx==0):
                    ad=0.01
                marker.scale.x = current_array.msgs[i].lx
                marker.scale.y = current_array.msgs[i].ly
                marker.scale.z = current_array.msgs[i].lz
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = current_array.msgs[i].x+ad
                marker.pose.position.y = current_array.msgs[i].y+ad
                marker.pose.position.z = current_array.msgs[i].z+ad
                print("x:",marker.pose.position.x)
                print("y:",marker.pose.position.y)
                print("z:",marker.pose.position.z)
                print("lx:",marker.scale.x)
                print("ly:",marker.scale.y)
                print("lz:",marker.scale.z)
                #print("\n x:",marker.scale.x)
                marker_text = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                pose=Pose(Point(marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z + marker.scale.z + 0.03),
                Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id="map"),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text=current_array.msgs[i].label.name)
		



                #print(marker)
                markerArray.markers.append(marker)
                for m in markerArray.markers:
                    m.id = i
                    i += 1
                markerArray_text.markers.append(marker_text)
                for m in markerArray_text.markers:
                    m.id = i
                    i += 1
            publisher_mark.publish(markerArray)
            publisher_text.publish(markerArray_text)
           
	    print("for loop:"+str((rospy.get_rostime()-now).to_sec()))
	    print("whole process:"+str((rospy.get_rostime()-start).to_sec()))
            id += 1
            #r.sleep()

        else:
            #r_max_id.sleep()
            publisher_mark.publish(markerArray)
            publisher_text.publish(markerArray_text)
	    print("whole process:"+str((rospy.get_rostime()-start).to_sec()))

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

