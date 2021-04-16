#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
from math import pi,pow,sqrt
from geometry_msgs.msg import PoseStamped,Point32
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import Float32,Float32MultiArray
from morai_msgs.msg import ObjectInfo
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from velocity_planning import vaildObject,cruiseControl
from scout_msgs.msg import ScoutStatus
class localPathPlanning :

    def __init__(self):
        rospy.init_node('localPathPlanning', anonymous=True)
        self.local_path_pub = rospy.Publisher('local_path',Path, queue_size=1)
        self.target_vel_pub = rospy.Publisher('target_vel',Float32, queue_size=1)
      
        rospy.Subscriber('/scout_status', ScoutStatus, self.scout_status_callback)
        rospy.Subscriber('/odom', Odometry, self.current_pose_callback)
        rospy.Subscriber('/global_path', Path, self.path_callback)
        rospy.Subscriber("/Object_topic", ObjectInfo, self.objectInfoCB)
        self.local_path_point_num=50
        self.is_pose=False
        self.is_path=False
        self.is_obj=False
        self.is_status=False
        self.target_vel_msg=Float32()

        vaild_obj=vaildObject()
        cruise_control=cruiseControl(0.5,1)

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            # print(self.is_path,self.is_pose,self.is_obj)
   
            if self.is_path and self.is_pose and self.is_obj:

                local_path=Path()
                x=self.pose_msg.pose.pose.position.x
                y=self.pose_msg.pose.pose.position.y
                current_waypoint=-1

                min_dis=float('inf')
                for i in range(len(self.path_msg.poses)) :
                    dx=x - self.path_msg.poses[i].pose.position.x
                    dy=y - self.path_msg.poses[i].pose.position.y
                    dis=sqrt(dx*dx + dy*dy)
                    if dis < min_dis :
                        min_dis=dis
                        current_waypoint=i


                if current_waypoint+self.local_path_point_num < len(self.path_msg.poses) :
                    end_waypoint= current_waypoint+self.local_path_point_num 

                else :
                    end_waypoint=len(self.path_msg.poses)


                local_path.header.frame_id='map'
                for i in range(current_waypoint,end_waypoint) :
                    tmp_pose=PoseStamped()
                    tmp_pose.pose.position.x=self.path_msg.poses[i].pose.position.x
                    tmp_pose.pose.position.y=self.path_msg.poses[i].pose.position.y
                    tmp_pose.pose.position.z=self.path_msg.poses[i].pose.position.z
                    tmp_pose.pose.orientation.x=0
                    tmp_pose.pose.orientation.y=0
                    tmp_pose.pose.orientation.z=0
                    tmp_pose.pose.orientation.w=1
                    local_path.poses.append(tmp_pose)
                
                vaild_obj.get_object(self.object_info_msg)
                global_vaild_object,local_vaild_object=vaild_obj.calc_vaild_obj([x,y,self.vehicle_yaw])
                print(global_vaild_object,local_vaild_object)
                cruise_control.checkObject(local_path,global_vaild_object,local_vaild_object)
                target_vel=cruise_control.acc(local_vaild_object,self.current_velocity,3.0)
                self.target_vel_msg.data=target_vel
                self.target_vel_pub.publish(self.target_vel_msg)
                self.local_path_pub.publish(local_path)
            
            rate.sleep()


    def current_pose_callback(self,msg):
        self.pose_msg=msg
        self.is_pose=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)


    def path_callback(self,msg):
        self.path_msg=msg
        self.is_path=True
    
       
    def objectInfoCB(self,data):
        self.is_obj=True
        self.object_info_msg=data

    def scout_status_callback(self,data):
        self.current_velocity=data.linear_velocity
        self.is_status=True

if __name__ == '__main__':
    
    test=localPathPlanning()



