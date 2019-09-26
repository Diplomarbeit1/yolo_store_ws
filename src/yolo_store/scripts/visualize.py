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
import numpy as np

###import my message
from tmc_vision_msgs.msg import DetectionArray , Detection, Label
from tmc_vision_msgs.msg import yolo_store_msg_Array, yolo_store_msg
from std_msgs.msg import Header , String
from std_msgs.msg import Int32

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import random


# topic = 'visualization_marker_array'
# publisher = rospy.Publisher(topic, MarkerArray,queue_size=1)

max_dist = 0.4      # default max distance between objects

table = 1           #store tables
grow_sphere = 1.5
grow_normal = 1.3   #adjusting the size
d = 2               #duration of marker lifetime
probboost = 1.4     #boost of probability
probthresh = 0.3    #threshold of probability 

def visualize():
    rospy.init_node('listener', anonymous=True)
    rospy.spin()


# rospy.init_node('register')

markerArray = MarkerArray()
markerArray_pub = MarkerArray()
markerArray_text = MarkerArray()
weight_arr = []
name_arr = []
count = 0.0
cnt = 0.0



id = 0
np.matrix([0])
i_ = 0 
j_ = 0

if __name__ == '__main__':
    rospy.init_node('visualize', anonymous=True)
    topic_mark = '/yolo_store/marker_array'
    topic_text = '/yolo_store/marker_text_array'
    topic_ind = '/yolo_store/max_index'
    publisher_mark = rospy.Publisher(topic_mark, MarkerArray,queue_size=1)
    publisher_text = rospy.Publisher(topic_text, MarkerArray,queue_size=1)
    publisher_1mark = rospy.Publisher('yolo_store/table', Marker,queue_size=1)
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

        #print("query:"+str((rospy.get_rostime()-now).to_sec()))
        now = rospy.get_rostime()

        #print((mongo_array[1][0].msgs[0]))
        if id < max_id & max_id > 0:
           
            current_array = mongo_array[id][0]
            i=0

            for i in range(current_array.msgs.__len__()):
                name = str(current_array.msgs[i].label.name)
                print("   visualize marker: " + name +" "+ str(i+1)+"/"+str(current_array.msgs.__len__()))
                store = 1
                #rospy.sleep(3)

                if(name=='cake' or name == 'person' or name == 'toilet' or name == 'chair' 
                or current_array.msgs[i].x==0 or current_array.msgs[i].lx==0 
                or current_array.msgs[i].label.confidence <probthresh):
                    store = 0                                               #these objects are not stored
                
                marker = Marker()
                marker.header.frame_id = "map"
                marker.type = marker.CUBE

                marker.action = marker.ADD
                if(name=="apple" or name=="orange" or name=="mouse" or name=="sports ball" ):
                    grow_factor = grow_sphere
                else:
                    grow_factor = grow_normal

                lx = current_array.msgs[i].lx * grow_factor 
                ly = current_array.msgs[i].ly * grow_factor 
                lz = current_array.msgs[i].lz * grow_factor * 0.8

                
                marker.color.r = 0#random.random()
                marker.color.g = 1#random.random()
                marker.color.b = 0#random.random()
                marker.pose.orientation.w = 1

                x = current_array.msgs[i].x
                y = current_array.msgs[i].y
                z = current_array.msgs[i].z   

                if((z-lz/2)<0):                 
                    lz = lz*0.8                     #object is under z=0
                    z = lz/2
                    
                if((name == 'table' or name == 'diningtable' or name == 'sofa') and marker.scale.z < 0.1): 
                    # lx = lx/grow_factor
                    # ly = ly/grow_factor
                    # lz = lz/grow_factor
       
                    height = z 
                    lz = height * 0.9                #object was detected as dz~=0 area and table/diningtable/sofa/chair
                    z = height/2                    #therfore it should reach bottom
                    probboost = 0.8
                    store = table

                prob = current_array.msgs[i].label.confidence *probboost
                if(prob>0.9): prob = 0.9
                marker.color.a = current_array.msgs[i].label.confidence
                
                weight = 1
                length_marker = markerArray.markers.__len__()

                if(length_marker > 0 and store and lx>0):

                    j = 0
                    while j < length_marker:  #range():
                        j+=1
                        distx = x-markerArray.markers[j-1].pose.position.x
                        disty = y-markerArray.markers[j-1].pose.position.y
                        distz = z-markerArray.markers[j-1].pose.position.z

                        dist = math.sqrt(math.pow((distx),2) + math.pow((disty),2) + math.pow((distz),2))

                        thresh_ = max_dist
                        if(thresh_>lx/2): thresh_ = lx/2                                                               #only search within object area 
                        if(thresh_>markerArray.markers[j-1].scale.x/2): thresh_ = markerArray.markers[j-1].scale.x/2   #within the bigger object area of the 2 objects
                        
                        if((dist <= thresh_) and lx<1.5 and ly<1.5 and lz<1.5):
                            name_o = name
                            name_n = markerArray_text.markers[j-1].text
                            if(name_o=="orange" or name_n=="orange"):
                                name_o="orange"
                            if(name_o=="mouse" or name_n=="mouse"): 
                                name_o="mouse"
                            name=name_o
  
                            lx = markerArray.markers[j-1].scale.x + (lx-markerArray.markers[j-1].scale.x) / (weight + weight_arr[j-1])
                            ly = markerArray.markers[j-1].scale.y + (ly-markerArray.markers[j-1].scale.y) / (weight + weight_arr[j-1])
                            lz = markerArray.markers[j-1].scale.z + (lz-markerArray.markers[j-1].scale.z) / (weight + weight_arr[j-1])
                            
                            x = markerArray.markers[j-1].pose.position.x + distx / (weight + weight_arr[j-1])
                            y = markerArray.markers[j-1].pose.position.y + disty / (weight + weight_arr[j-1])
                            z = markerArray.markers[j-1].pose.position.z + distz / (weight + weight_arr[j-1])  

                            weight += weight_arr[j-1] 

                            del weight_arr[j-1]
                            del markerArray.markers[j-1]
                            del markerArray_text.markers[j-1]

                            length_marker = markerArray.markers.__len__()
                        
                if(store  and lx<1.5 and ly<1.5 and lz<1.5 and x!=0 and y!=0):

                    if(name=="apple" or name=="orange" or name=="mouse" or name=="sports ball" ):
                        marker.type = marker.SPHERE
                        marker.color.r = 1
                        marker.color.g = 0
                        marker.color.b = 0
                    if(name=="cup" or name=="bottle" or name=="vase" or name=="pottedplant"):
                        marker.type = marker.CYLINDER
                        marker.color.r = 0
                        marker.color.g = 0
                        marker.color.b = 1
                    if(marker.type==2):
                        lz = lx
                    if(name=="apple"):
                        marker.color.r = 1
                        marker.color.g = 0
                        marker.color.b = 0
                    if(name=="orange"):
                        marker.color.r = 1
                        marker.color.g = 0.5
                        marker.color.b = 0

                    marker.scale.x = lx
                    marker.scale.y = ly
                    marker.scale.z = lz

                    marker.pose.position.x = x
                    marker.pose.position.y = y
                    marker.pose.position.z = z

                    weight_arr.append(1 + weight)

                    marker_text = Marker(
                    type = Marker.TEXT_VIEW_FACING,
                    id = 0,
                    pose=Pose(Point(x, y, z + marker.scale.z/2 + 0.05),
                    Quaternion(0, 0, 0, 1)),
                    scale = Vector3(0.06, 0.06, 0.06),
                    header = Header(frame_id="map"),
                    color = ColorRGBA(marker.color.r, marker.color.g, marker.color.b ,1.0),
                    text = name)


                    t = rospy.Duration(d)
                    marker.lifetime = t
                    marker_text.lifetime = t
                    
                    marker.id = i_
                    markerArray.markers.append(marker)
                    marker_text.id = i_
                    markerArray_text.markers.append(marker_text)

                    i_ += 1

                else: 
                    print("         "+str(name)+" was not stored")
                    del marker
                # else: 
                #     print("###THIS IS NOT RIGHT: PERSON OR TOILET")
            

            #markerArray_pub.markers = markerArray.markers[0:(markerArray.markers.__len__()-2)]
            #print(markerArray.markers.__len__())
            publisher_mark.publish(markerArray)
            
            publisher_text.publish(markerArray_text)
            #rospy.sleep(d)
            #print("for loop:"+str((rospy.get_rostime()-now).to_sec()))
            #print("whole process:"+str((rospy.get_rostime()-start).to_sec()))
            id += 1
            #rospy.sleep(5)

        else:
	    #np.matrix([[1,2],[2,4]])
            #rospy.sleep(1)
            # markerArray_pub.markers = markerArray.markers[0:(markerArray.markers.__len__()-2)]
            # print(markerArray.markers.__len__())
            publisher_mark.publish(markerArray)
            #publisher_mark.publish(markerArray)
            publisher_text.publish(markerArray_text)
            #print("whole process:"+str((rospy.get_rostime()-start).to_sec()))



