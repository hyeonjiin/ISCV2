#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import rospkg
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import Float64,Int16,Float32MultiArray
import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi
import tf
from morai_msgs.msg import CtrlCmd
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class pathReader :
    def __init__(self,pkg_name):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)



    def read_txt(self,file_name):
        full_file_name=self.file_path+"/path/"+file_name
        openFile = open(full_file_name, 'r')
        print(full_file_name)
        out_path=Path()
        
        out_path.header.frame_id='/map'
        line=openFile.readlines()
        for i in line :
            tmp=i.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.position.z=float(tmp[2])
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=1
            out_path.poses.append(read_pose)
        
        
        openFile.close()
        return out_path
      



def findLocalPath(ref_path,status_msg,start):
    out_path=Path()
    current_x=status_msg.pose.pose.position.x
    current_y=status_msg.pose.pose.position.y
    current_waypoint=0
    min_dis=float('inf')
    #T자 인지
    destiny=start+300
    if destiny>=len(ref_path.poses):
        destiny = len(ref_path.poses)

    for i in range(start,destiny) :
    #for i in range(0,len(ref_path.poses)) :
        dx=current_x - ref_path.poses[i].pose.position.x
        dy=current_y - ref_path.poses[i].pose.position.y
        dis=sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i
    print('---------------',current_waypoint)


    if current_waypoint+30 > len(ref_path.poses) :
        last_local_waypoint= len(ref_path.poses)
    else :
        last_local_waypoint=current_waypoint+30



    out_path.header.frame_id='map'
    for i in range(current_waypoint,last_local_waypoint) :
        tmp_pose=PoseStamped()
        tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x=0
        tmp_pose.pose.orientation.y=0
        tmp_pose.pose.orientation.z=0
        tmp_pose.pose.orientation.w=1
        out_path.poses.append(tmp_pose)
    
    return out_path,current_waypoint

#class purePursuit :
#    def __init__(self):
#        
#        self.is_path=False
#        self.is_odom=False
#        
#        self.forward_point=Point()
#        self.current_postion=Point()
#        self.is_look_forward_point=False
#        self.lfd= 3
#        self.min_lfd= 2.9
#        self.max_lfd= 3.1
#        self.vehicle_length= 1
#        self.steering=0
#        
#    def getPath(self,msg):
#        self.path=msg  #nav_msgs/Path 
#    
#    
#    def getEgoStatus(self,msg):
#
#        self.current_vel=20  #kph
#        self.is_odom = True
#        
#        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
#        _,_,self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
#        self.current_postion.x = msg.pose.pose.position.x
#        self.current_postion.y = msg.pose.pose.position.y
#        self.current_postion.z = 0
#
#    def steering_angle(self):
#        vehicle_position=self.current_postion
#        rotated_point=Point()
#        self.is_look_forward_point= False
#
#        
#
#        for i in self.path.poses :
#            path_point=i.pose.position
#            dx= path_point.x - vehicle_position.x
#            dy= path_point.y - vehicle_position.y
#            rotated_point.x=cos(self.vehicle_yaw)*dx +sin(self.vehicle_yaw)*dy
#            rotated_point.y=sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
# 
#            
#            if rotated_point.x>0 :
#                dis=sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
#                
#                print("dis , lfd : ", dis, self.lfd)
#                if dis>= self.lfd :
#                    
#                    self.lfd=self.current_vel / 1.8
#                    if self.lfd < self.min_lfd : 
#                        self.lfd=self.min_lfd
#                    elif self.lfd > self.max_lfd :
#                        self.lfd=self.max_lfd
#                    self.forward_point=path_point
#                    self.is_look_forward_point=True
#                    
#                    break
#        
#        theta=atan2(rotated_point.y,rotated_point.x)
#
#        if self.is_look_forward_point :
#            self.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd) #rad
#            print(self.steering)
#            return self.steering 
#        else : 
#            print("no found forward point")
#            return 0

                     
class purePursuit :
    def __init__(self):
        
        self.is_path=False
        self.is_odom=False
        
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.lfd= 3.0
        self.min_lfd= 0.5
        self.max_lfd= 3
        self.vehicle_length= 4.635
        self.steering=0
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=2
        self.error=0
        
    def getPath(self,msg):
        self.is_path = True
        self.path=msg  #nav_msgs/Path 
    
    
    def getEgoStatus(self,msg):
        self.current_vel=20  #kph
        self.is_odom = True
        
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y
        self.current_postion.z = 0

    def steering_angle(self,current_vel, error):
        vehicle_position=self.current_postion
        rotated_point=Point()
        self.is_look_forward_point= False
        
        vehicle_position=self.current_postion
        self.is_look_forward_point = False
        translation = [vehicle_position.x , vehicle_position.y]
        if error==1:
            self.vehicle_yaw+=180

        t = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0                    , 0                     , 0 ]
        ])
        
        det_t = np.array([
            [t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])],
            [t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])],
            [0      ,0      , 1]
        ])
        #print(self.path.poses)
        for num,i in enumerate(self.path.poses) :
            path_point=i.pose.position
            
            global_path_point = [path_point.x,path_point.y,1]
            local_path_point = det_t.dot(global_path_point)
            if local_path_point[0]>0 :
                dis=sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                if dis>= self.lfd:
                    self.lfd=current_vel / 5
                    self.forward_point=path_point
                    self.is_look_forward_point = True
                    break
        
        theta=atan2(local_path_point[1],local_path_point[0])
        if self.is_look_forward_point :
            self.ctrl_cmd_msg.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd)
        #     #print(self.ctrl_cmd_msg.steering)
        else :
            print("no found forward point")
            if error==0:
                error=1
            elif error==1:
                error=2

            self.ctrl_cmd_msg.steering = 0.0
            self.ctrl_cmd_msg.velocity = 0.0
        self.ctrl_cmd_msg.steering/=2
        self.ctrl_cmd_msg.steering =  round(self.ctrl_cmd_msg.steering,2)
        print(self.ctrl_cmd_msg.steering)
        #print(self.ctrl_cmd_msg.steering)
        return self.ctrl_cmd_msg.steering, error   



