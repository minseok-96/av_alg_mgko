#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import Float64,Int16
import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi
from morai_msgs.msg import ObjectInfo



class cruiseControl:
    def __init__(self,object_vel_gain,object_dis_gain):
        self.object=[False,0]
        self.object_vel_gain=object_vel_gain
        self.object_dis_gain=object_dis_gain


    def checkObject(self,ref_path,global_vaild_object,local_vaild_object):
        self.object=[False,0]
    
        if len(global_vaild_object) >0  :
            min_rel_distance=float('inf')

            for i in range(len(global_vaild_object)):
                
                for path in ref_path.poses :

                    if global_vaild_object[i][0]==1 or  global_vaild_object[i][0]==2:

                        dis=sqrt(pow(path.pose.position.x-global_vaild_object[i][1],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))
                        
                        if dis<2 :
                            rel_distance= sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance=rel_distance
                                self.object=[True,i]

                            break
                    




    def acc(self,local_vaild_object,ego_vel,target_vel):
        out_vel=target_vel
        # print(self.object)
        if self.object[0] == True :
            print("ACC ON")   
            front_vehicle=[local_vaild_object[self.object[1]][1],local_vaild_object[self.object[1]][2],local_vaild_object[self.object[1]][3]]
            time_gap=0.3  
            default_space=1.5
            dis_safe=ego_vel* time_gap+default_space
            dis_rel=sqrt(pow(front_vehicle[0],2)+pow(front_vehicle[1],2))-2
            # print(dis_rel)
            vel_rel=(front_vehicle[2]-ego_vel)  
            v_gain=self.object_vel_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)    

            acc_based_vel=ego_vel+acceleration

            if acc_based_vel > target_vel : 
                acc_based_vel=target_vel
            
            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel
            else :
                if acc_based_vel<target_vel :
                    out_vel=acc_based_vel
            
        return out_vel

class vaildObject :

    def __init__(self):
        pass
    def get_object(self,obj_msg):
        self.all_object=ObjectInfo()
        self.all_object=obj_msg

    def calc_vaild_obj(self,ego_pose):
        global_object_info=[]
        loal_object_info=[]
        if self.all_object.num_of_objects > 0:

            tmp_theta=ego_pose[2]
            tmp_translation=[ego_pose[0],ego_pose[1]]
            tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],
                            [sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],
                            [0,0,1]])
            tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])   ],
                                [tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])   ],
                                [0,0,1]])

            for num in range(self.all_object.num_of_objects):
                global_result=np.array([[self.all_object.pose_x[num]],[self.all_object.pose_y[num]],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]>0 :
                    global_object_info.append([self.all_object.object_type[num],self.all_object.pose_x[num],self.all_object.pose_y[num],self.all_object.velocity[num]])
                    loal_object_info.append([self.all_object.object_type[num],local_result[0][0],local_result[1][0],self.all_object.velocity[num]])
            
                   
        return global_object_info,loal_object_info

