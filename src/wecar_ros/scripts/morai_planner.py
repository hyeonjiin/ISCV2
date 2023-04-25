#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys,os
import rospy
import rospkg
import math
import time
from sensor_msgs.msg import Imu
from morai_msgs.srv import MoraiEventCmdSrv, MoraiEventCmdSrvRequest
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,CtrlCmd,GetTrafficLightStatus,SetTrafficLight
from lib.utils import pathReader, findLocalPath,purePursuit
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import PointCloud
from math import cos,sin,sqrt,pow,atan2,pi
from pyproj import Proj
import time


class morai_planner():
    def __init__(self):
        rospy.init_node('morai_planner', anonymous=True)
        rospy.wait_for_service("/Service_MoraiEventCmd")
        self.service_client = rospy.ServiceProxy("/Service_MoraiEventCmd",MoraiEventCmdSrv)
        self.request_srv = MoraiEventCmdSrvRequest()
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size= 1)
        self.camera_sub = rospy.Subscriber("/camera_steering",CtrlCmd,self.line_callback)
        self.traffic_sub = rospy.Subscriber("/camera_traffic", CtrlCmd,self.traffic_callback)
        self.corner_sub = rospy.Subscriber("/corner_steering", CtrlCmd,self.corner_callback)
        self.br = tf.TransformBroadcaster()
    
        self.x, self.y = None, None
        self.is_imu = False
        self.gps_imu = False
        self.proj_UTM = Proj(proj='utm',zone=52,ellps='WGS84', preserve_units = False)
        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'
        self.odom_msg.child_frame_id='/base_link1'
           
        #text파일 입력
        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]
        
        #publisher / subscriber 모음
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size = 1)
        #rospy.Subscriber("odom",Odometry,self.odom_callback)        
        
        #기본정의
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=2
        self.steering = 0
        self.velocity = 15

        #class
        pure_pursuit=purePursuit() ## purePursuit import
        self.future_waypoint = 0 #방문한 path check 안하기
        self.original = 0

        #camera lane
        self.camera_lane_msg = CtrlCmd()
        self.camera_lane_msg.longlCmdType=2
       

        #traffic
        self.traffic_msg = CtrlCmd()
        self.camera_lane_msg.longlCmdType=2

        
        #corner lane
        self.corner_lane_msg = CtrlCmd()
        self.corner_lane_msg.longlCmdType=2


        #t자 주차 후진, 전진
        self.is_rear = False
        self.is_front = False

        #read path
        self.path_reader=pathReader('wecar_ros')
        self.global_path=self.path_reader.read_txt(self.path_name+".txt")
        self.mission = 0
        self.error = 0
        self.past_error = 0
        self.back = 0

        #음영구간 확인
        self.dark = 0

        #오르막길 미션 확인
        self.up = 0

        
        
        #self.global_path 자체가 nav_msgs/Path 타입을 가지고 있다.
        #그래서 바로 publish해주었다!
        #nav_msgs/path는 global_path.poses[1].pose.position 이런식으로 접근해야 한다.
        #그래서 pure_pursuit.py에서 for문을 돌릴 때 이 부분을 확인할 수 있다.
        

        rate = rospy.Rate(30) # 30hz

        while not rospy.is_shutdown():
            
            local_path,self.current_waypoint=findLocalPath(self.global_path,self.odom_msg,self.future_waypoint)
            print('current 위치 : ',self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, self.current_waypoint )

            self.convertLL2UTM() 
            pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용.camera_lane_msg
            pure_pursuit.getEgoStatus(self.odom_msg) ## pure_pursuit 알고리즘에 차량의 status 적용
            
            self.steering,self.error=pure_pursuit.steering_angle(self.velocity,self.error)
            self.future_waypoint = self.current_waypoint


            
            #S자 주행, ㄱ자
            if self.lat==0 and self.lon==0:
                if self.dark == 0:
                    #self.darkness()
                    self.ctrl_cmd_pub.publish(self.corner_lane_msg)
                else:
                    self.ctrl_cmd_pub.publish(self.camera_lane_msg)
            
            else:
                if self.mission==0:
                    if abs(self.steering)>=0.25:
                        self.velocity = 8
                    else:
                        self.velocity=15
                    self.ctrl_cmd_msg.steering = self.steering
                    self.ctrl_cmd_msg.velocity = self.velocity
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)    
            
            #출발 깜박이키기
            if self.odom_msg.pose.pose.position.x>=402468 and self.odom_msg.pose.pose.position.x<=402471 and self.odom_msg.pose.pose.position.y>=4132979 and self.odom_msg.pose.pose.position.y<=4132981:
                self.control_change()
                self.request_srv.request.lamps.turnSignal = 1
                self.service_client(self.request_srv)

            if self.odom_msg.pose.pose.position.x>=402469 and self.odom_msg.pose.pose.position.x<=402472 and self.odom_msg.pose.pose.position.y>=4132985 and self.odom_msg.pose.pose.position.y<=4132987:
                self.control_change()
                self.request_srv.request.lamps.turnSignal = 0
                self.service_client(self.request_srv)
            
            #오르막길
            if self.up == 0 and self.odom_msg.pose.pose.position.x>=402477 and self.odom_msg.pose.pose.position.x<=402481 and self.odom_msg.pose.pose.position.y>=4133025.5 and self.odom_msg.pose.pose.position.y<=4133027.5:
                self.ctrl_cmd_msg.brake = 1
                self.ctrl_cmd_msg.velocity=0
                print(self.ctrl_cmd_msg)
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  
                for i in range(4):
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg) 
                    time.sleep(1)

                self.ctrl_cmd_msg.brake = 0
                self.up=1

            #신호등
                #직진
            if self.odom_msg.pose.pose.position.x>=402456.5 and self.odom_msg.pose.pose.position.x<=402458.2 and self.odom_msg.pose.pose.position.y>=4133026.2 and self.odom_msg.pose.pose.position.y<=4133029:
                self.dark=1
                if self.traffic_msg.brake == 1 :
                    self.traffic_msg.velocity = 0
                    self.ctrl_cmd_pub.publish(self.traffic_msg)
                else:
                    self.ctrl_cmd_msg.steering = self.steering
                    self.ctrl_cmd_msg.velocity = 13
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                #좌회전
            if self.odom_msg.pose.pose.position.x>=402426.6 and self.odom_msg.pose.pose.position.x<=402429 and self.odom_msg.pose.pose.position.y>=4133027 and self.odom_msg.pose.pose.position.y<=4133031:
                if self.traffic_msg.brake == 1 :
                    self.ctrl_cmd_pub.publish(self.traffic_msg)
                    self.control_change()
                    self.request_srv.request.lamps.turnSignal = 1
                    self.traffic_msg.velocity = 0
                    self.service_client(self.request_srv)
                    self.ctrl_cmd_pub.publish(self.traffic_msg)
                else:
                    self.control_change()
                    self.request_srv.request.lamps.turnSignal = 1
                    self.ctrl_cmd_msg.steering = self.steering
                    self.ctrl_cmd_msg.velocity = 13
                    self.service_client(self.request_srv)
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                #좌회전후 신호 끄기
            if self.odom_msg.pose.pose.position.x>=402445 and self.odom_msg.pose.pose.position.x<=402448 and self.odom_msg.pose.pose.position.y>=4133036 and self.odom_msg.pose.pose.position.y<=4133039:
                self.control_change()
                self.request_srv.request.lamps.turnSignal = 0
                self.service_client(self.request_srv)
                #직진
            if self.odom_msg.pose.pose.position.x>=402442 and self.odom_msg.pose.pose.position.x<=402444 and self.odom_msg.pose.pose.position.y>=4133043.5 and self.odom_msg.pose.pose.position.y<=4133045.2:
                if self.traffic_msg.brake == 1 :
                    self.traffic_msg.steering = self.steering
                    self.traffic_msg.velocity = 0
                    self.ctrl_cmd_pub.publish(self.traffic_msg)
                else:
                    self.ctrl_cmd_msg.steering = self.steering
                    self.ctrl_cmd_msg.velocity = 13
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)    
            #가속
            if self.odom_msg.pose.pose.position.x>=402410 and self.odom_msg.pose.pose.position.x<=402413.3 and self.odom_msg.pose.pose.position.y>=4133073.4 and self.odom_msg.pose.pose.position.y<=4133075.0:  
                self.ctrl_cmd_msg.longlCmdType=1
                self.ctrl_cmd_msg.accel=0.7
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)    
            
            #감속
            if self.odom_msg.pose.pose.position.x>=402404 and self.odom_msg.pose.pose.position.x<=402408 and self.odom_msg.pose.pose.position.y>=4133043.0 and self.odom_msg.pose.pose.position.y<=4133046.0:  
                self.ctrl_cmd_msg.longlCmdType=2
                self.ctrl_cmd_msg.steering = self.steering
                self.ctrl_cmd_msg.velocity = self.velocity
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)    

            #T자를 위해서

            if self.mission==1:
                print(self.error,'----------------------------------')
                self.velocity = 3.0
                if self.odom_msg.pose.pose.position.x>=402404 and self.odom_msg.pose.pose.position.x<=402409 and self.odom_msg.pose.pose.position.y>=4133028 and self.odom_msg.pose.pose.position.y<=4133030:
                    self.mission = 2
                if self.error == 0:
                    self.ctrl_cmd_msg.steering = self.steering
                    self.ctrl_cmd_msg.velocity = self.velocity
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                
                elif self.error==1:
                    print('error==1111111111',self.is_rear)
                    self.global_path=self.path_reader.read_txt("test_path3.txt")
                    self.future_waypoint = 0
                    if self.is_rear == True :
                        self.gear_change(self.error)
                        self.ctrl_cmd_msg.steering = self.steering*-1
                        self.ctrl_cmd_msg.velocity = 3
                        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    elif self.is_rear == False :
                        self.velocity = 0
                        self.ctrl_cmd_msg.velocity = self.velocity
                        self.is_rear = True
                        for i in range(0,100000):
                            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                
                elif self.error==2:
                    print('error==22222222',self.is_front)
                    self.global_path=self.path_reader.read_txt("test_path4.txt")
                    self.future_waypoint = 0
                    if self.is_front == True :
                        self.gear_change(self.error)
                        self.ctrl_cmd_msg.steering = self.steering
                        self.ctrl_cmd_msg.velocity = 3
                        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

                    elif self.is_front == False :
                        self.velocity = 0
                        self.ctrl_cmd_msg.velocity = self.velocity
                        self.is_front = True
                        for i in range(0,100000):
                            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            #T자 나와서 path switching
            if self.mission==2:
                self.global_path=self.path_reader.read_txt("final_lap.txt")
                self.future_waypoint = self.original
                self.mission=0
            
            #T자진입 - path switching 
            if self.mission==0 and self.odom_msg.pose.pose.position.x>=402404 and self.odom_msg.pose.pose.position.x<=402409 and self.odom_msg.pose.pose.position.y>=4133013 and self.odom_msg.pose.pose.position.y<=4133016:
                self.original= self.current_waypoint
                self.global_path=self.path_reader.read_txt("test_path2.txt")
                self.mission=1
                self.error=0
                self.past_error=0
                self.future_waypoint = 0
            
            if self.current_waypoint >= 2200 and self. current_waypoint<=2219:
                self.control_change()
                self.request_srv.request.lamps.turnSignal = 2
                self.service_client(self.request_srv)
            
            # 종료전 우측 깜박이 켜기
            if self.current_waypoint >= 2200 and self. current_waypoint<=2219:
                self.control_change()
                self.request_srv.request.lamps.turnSignal = 2
                self.service_client(self.request_srv)
            
            #종료 후 parking으로 바꾸기
            if self.current_waypoint>=2240:
                self.ctrl_cmd_msg.brake = 1
                self.ctrl_cmd_msg.velocity=0
                self.ctrl_cmd_msg.steering=0
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  
                
                self.request_srv.request.option = 2
                self.request_srv.request.gear=1
                self.service_client(self.request_srv)


            
            
                    
            


            #self.ctrl_cmd_msg.steering = self.steering
            #self.ctrl_cmd_msg.velocity = self.velocity
            
            #self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            
            #print(self.camera_lane_msg)
            #self.ctrl_cmd_pub.publish(self.camera_lane_msg)

            rate.sleep()    

    def gear_change(self,error):
        self.request_srv.request.option = 2
        if error==1:
            self.request_srv.request.gear=2
        elif error==2:
            self.request_srv.request.gear=4
        result = self.service_client(self.request_srv)
        print(result)
    
    def control_change(self):
        self.request_srv.request.option = 6
        self.request_srv.request.gear=4

    def darkness(self):
        for i in range(2):
            self.ctrl_cmd_msg.steering = 0.0
            self.ctrl_cmd_msg.velocity = 7
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            time.sleep(1)
        for i in range(7):
            self.ctrl_cmd_msg.steering = 45
            self.ctrl_cmd_msg.velocity = 5
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            time.sleep(1)
        for i in range(2):
            self.ctrl_cmd_msg.steering = 0.0
            self.ctrl_cmd_msg.velocity = 4.5
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            time.sleep(1)
        for i in range(7):
            self.ctrl_cmd_msg.steering = -45
            self.ctrl_cmd_msg.velocity = 5
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            time.sleep(1)
        while(1):
            #print(self.camera_lane_msg)
            self.camera_lane_msg.velocity=6
            self.ctrl_cmd_pub.publish(self.camera_lane_msg)
            if self.lat!=0:
                break

    def navsat_callback(self, gps_msg):
        self.gps_imu = True
        
        self.latitude = gps_msg.latitude
        self.longitude = gps_msg.longitude
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        
        self.convertLL2UTM()
        
        
        br = tf.TransformBroadcaster()
        br.sendTransform((self.x, self.y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "gps",
                        "map")
        
        self.utm_msg = Float32MultiArray()
        
        self.utm_msg.data = [self.x , self.y]
        
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0
        
        self.odom_pub.publish(self.odom_msg)
    
    def convertLL2UTM(self):
        
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x = xy_zone[0]
        self.y = xy_zone[1]
        
    def imu_callback(self, data):
        self.is_imu = True
        
        self.odom_msg.pose.pose.orientation.x = data.orientation.x
        self.odom_msg.pose.pose.orientation.y = data.orientation.y
        self.odom_msg.pose.pose.orientation.z = data.orientation.z
        self.odom_msg.pose.pose.orientation.w = data.orientation.w
        
        quaternion=(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)
        roll,pitch,yaw = euler_from_quaternion(quaternion)
        self.roll_deg=roll/pi*180
        self.pitch_deg=pitch/pi*180
        self.yaw_deg=yaw/pi*180
        
        self.prev_time = rospy.get_rostime()
    
    def line_callback(self, data):
        self.camera_lane_msg.steering = data.steering
        self.camera_lane_msg.velocity = data.velocity
    
    def traffic_callback(self, data):

        self.traffic_msg.brake = data.brake
    
    def corner_callback(self, data):
        self.corner_lane_msg.steering = data.steering
        self.corner_lane_msg.velocity = data.velocity



if __name__ == '__main__':
    try:
        kcity_pathtracking=morai_planner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass