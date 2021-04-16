#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
from math import pi,sqrt,pow
from nav_msgs.msg import Path
import numpy as np

class velocityPlanning :
    def __init__(self,car_max_speed,road_friction,point_num):
        self.car_max_speed=car_max_speed
        self.road_friction=road_friction
        self.point_num=point_num

    def curveBasedVelocity(self,global_path):
        out_vel_plan=[]
        for i in range(0,self.point_num):
            out_vel_plan.append(self.car_max_speed)


        for i in range(self.point_num,len(global_path.poses)-self.point_num):
            x_list=[]
            y_list=[]
            for box in  range(-self.point_num,self.point_num):
                x=global_path.poses[i+box].pose.position.x
                y=global_path.poses[i+box].pose.position.y
                x_list.append([-2*x,-2*y,1])
                y_list.append(-(x*x)-(y*y))
            
            x_matrix=np.array(x_list)
            y_matrix=np.array(y_list)
            x_trans=x_matrix.T
            

            a_matrix=np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a=a_matrix[0]
            b=a_matrix[1]
            c=a_matrix[2]
            r=sqrt(a*a+b*b-c)
            v_max=sqrt(r*9.8*self.road_friction)
            if v_max>self.car_max_speed :
                v_max=self.car_max_speed
            out_vel_plan.append(v_max)


        for i in range(len(global_path.poses)-self.point_num,len(global_path.poses)):
            out_vel_plan.append(0)
        
        return out_vel_plan
