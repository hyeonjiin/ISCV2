#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
import numpy as np
from morai_msgs.msg  import GPSMessage
from std_msgs.msg import Float32MultiArray
from pyproj import Proj
from math import pi,cos,sin,pi,sqrt,pow
from nav_msgs.msg import Path
import tf
from geometry_msgs.msg import PoseStamped


class test :

    def __init__(self):
        rospy.init_node('path_maker', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_folder_name=arg[1]
        self.make_path_name=arg[2]
        
        
        rospy.Subscriber("/gps",GPSMessage, self.status_callback)
        self.x , self.y = None, None
        self.proj_UTM = Proj(proj = 'utm', zone = 52, ellps = 'WGS84', preserve_units=False)
        
        

        self.is_status=False
        self.prev_x = 0
        self.prev_y = 0

        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('wecar_ros')
        full_path=pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'
        self.f=open(full_path, 'w')
        #print(self.f)

        rate=rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_status==True :
                self.path_make()
            rate.sleep()    

        self.f.close()
        

    def path_make(self):
        x=self.x
        y=self.y
        z=0
        distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
        print('distance-----------',distance)
        #0.1로 시도 - t자 주차
        #원래 - 0.3
        if distance > 0.3:
            data='{0}\t{1}\t{2}\n'.format(x,y,z)
            self.f.write(data)
            self.prev_x=x
            self.prev_y=y
            print(x,y)

    def status_callback(self,gps_msg): ## Vehicl Status Subscriber 
        self.is_status=True
        #self.status_msg=msg
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset
        
        self.convertLL2UTM()
        
        br = tf.TransformBroadcaster()
        br.sendTransform((self.x, self.y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "gps",
                        "map")
        
        utm_msg = Float32MultiArray()
        
        utm_msg.data = [self.x , self.y]
        #print(utm_msg)
    
    def convertLL2UTM(self):
        
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x = xy_zone[0]
        self.y = xy_zone[1]
        

if __name__ == '__main__':
    try:
        test_track=test()
    except rospy.ROSInterruptException:
        pass

