#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point32,PoseStamped,Point
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd
from tf.transformations import euler_from_quaternion,quaternion_from_euler


class output_gate :
    def __init__(self):
        rospy.init_node('outputGate', anonymous=True)
        rospy.Subscriber("/ctrl_lateral", CtrlCmd, self.ctrl_lateral_callback)   
        rospy.Subscriber("/ctrl_long", CtrlCmd, self.ctrl_long_callback)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)

        self.is_ctrl_lateral=False
        self.ctrl_msg=CtrlCmd()

        rospy.spin()
    
                


    def ctrl_long_callback(self,msg):
        
        if self.is_ctrl_lateral==True :
            
            accel=msg.accel   
            brake=msg.brake

            self.ctrl_msg.accel=accel
            self.ctrl_msg.brake=brake
            self.ctrl_msg.steering=self.steering
            self.ctrl_pub.publish(self.ctrl_msg)


    def ctrl_lateral_callback(self,msg):
        self.steering=msg.steering
        self.is_ctrl_lateral=True
        
if __name__ == '__main__':

    test=output_gate()

