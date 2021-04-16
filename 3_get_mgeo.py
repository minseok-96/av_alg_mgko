#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
from math import pi
from morai_msgs.msg  import EgoVehicleStatus
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Point32
import tf
from sensor_msgs.msg import PointCloud

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
# sys.path.append(os.path.normpath(os.path.join(current_path, 'mgko_alg/'))) 

mgeo_lib_path = os.path.normpath(os.path.join(current_path, './lib/mgeo/'))
# sys.path.append(mgeo_lib_path)

# from class_defs import * 

from lib.mgeo.class_defs import *
# import lib.mgeo.class_defs.mgeo_planner_map 



class test :

    def __init__(self):
        rospy.init_node('test', anonymous=True)
        self.link_pub = rospy.Publisher('link',PointCloud, queue_size=1)
        self.node_pub = rospy.Publisher('node',PointCloud, queue_size=1)


        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity'))
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes=node_set.nodes
        self.links=link_set.lines
        self.link_msg=self.getAllLinks()
        self.node_msg=self.getAllNode()
        print('# of nodes: ', len(node_set.nodes))
        print('# of links: ', len(link_set.lines))

       
      

        rate = rospy.Rate(1) # 20hz
        while not rospy.is_shutdown():
   
            self.link_pub.publish(self.link_msg)
            self.node_pub.publish(self.node_msg)

            rate.sleep()

 
    def getAllLinks(self):
        all_link=PointCloud()
        all_link.header.frame_id='map'
        
        for link_idx in self.links :
            for link_point in self.links[link_idx].points:
                tmp_point=Point32()
                tmp_point.x=link_point[0]
                tmp_point.y=link_point[1]
                tmp_point.z=0
                all_link.points.append(tmp_point)

        return all_link
    
    def getAllNode(self):
        all_node=PointCloud()
        all_node.header.frame_id='map'
        for node_idx in self.nodes :
            tmp_point=Point32()
            tmp_point.x=self.nodes[node_idx].point[0]
            tmp_point.y=self.nodes[node_idx].point[1]
            tmp_point.z=0
            all_node.points.append(tmp_point)

        return all_node


if __name__ == '__main__':
    
    test_track=test()



