#!/usr/bin/env python
###edit by chri
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations

import smach_ros
import rospy
import smach
import random

import control_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg

import tmc_msgs.msg._TalkRequestFeedback #used to create talk actions
import tmc_msgs.msg._TalkRequestGoal
import tmc_msgs.msg._TalkRequestAction
import tmc_msgs.msg._TalkRequestActionResult
import tmc_msgs.msg._TalkRequestResult
import tmc_msgs.msg._TalkRequestActionGoal
import tmc_msgs.msg._TalkRequestActionFeedback


#http://wiki.ros.org/mongodb_store
#roslaunch mongodb_store mongodb_store.launch  db_path:=/home/v4r/Chri/yolo_detection_db db_port:=62345


#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
import tf2_ros
import tf2_msgs.msg
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion

###import my message
from tmc_vision_msgs.msg import DetectionArray , Detection, Label
from tmc_vision_msgs.msg import yolo_store_msg_Array, yolo_store_msg
from std_msgs.msg import Header , String
import StringIO

def callback(data): 
	rospy.loginfo("I got detections")
	p_id = msg_store.insert_named(data.header.frame_id, data)
	
    
if __name__ == '__main__':
    rospy.init_node("message_store_client")
    msg_store = MessageStoreProxy()

    try:
	rospy.loginfo("waiting for detections")
	rospy.Subscriber("/yolo_store/msg_mongo",yolo_store_msg_Array,callback)
	rospy.spin()
	

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
