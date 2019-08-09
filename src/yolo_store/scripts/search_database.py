#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
from object_mapping.msg import DetectedObject3D
import StringIO


if __name__ == '__main__':
    rospy.init_node("example_message_store_client")


    msg_store = MessageStoreProxy()


    p = Pose(Point(0, 1, 2), Quaternion(3, 4,  5, 6))

    try:
	name=input("input name")
	pos=input("want to enter position? (y/n)"
	p=Point()
	if(pos)
		p.x=("Position x")
		p.y=("Position y")
		p.z=("Position z")
	 
	
        # some other things you can do...

        # get it back with a name
        #print msg_store.query_named("my favourite pose", Pose._type)


        # try to get it back with an incorrect name, so get None instead
        #print msg_store.query_named("my favourite position", Pose._type)

        # get all poses  
        #print msg_store.query(Pose._type, sort_query=[("$natural", -1)]) #, {"position.x":7.84818831388})
        #liste=( msg_store.query(Pose._type)
        #print liste[1]
        # get the latest one pose
        #print msg_store.query(Pose._type, sort_query=[("$natural", -1)], single=True)


        # get all non-existant typed objects, so get an empty list back
        #print msg_store.query( "not my type")
        
        # get all poses where the y position is 1
        #print msg_store.query(Pose._type, {"position.y":1})
        
        # get all poses where the y position greater than 0
        #print msg_store.query(Pose._type, {"position.y": {"$gt": 0}}  )# && "position.x": {"$gt": 0}})
        #print msg_store.query('chair')
        #list=(msg_store.query("person"))
        #liste=msg_store.query(Pose._type)
        #liste=msg_store.query_named("chair", Pose._type,1)

        #liste=msg_store.query_named("chair",Pose._type,0)
        list= msg_store.query_named("chair",DetectedObject3D._type,0)
        print list[0][0]
        #liste.encode('ascii','ignore')
        #print dir(liste)
        #print liste.append('chair')
        #for()
        #print liste[0][0].position.x
        #print liste[0][0].position.y
        #print liste[0][0].position.z
        #print (liste[1][1]['name'])
        #print dir(liste)#.First()
        #print liste[1]()
        #print list.index('chair')
        #print list.append(position)
    
    
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        
