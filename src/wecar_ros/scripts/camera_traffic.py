#!/usr/bin/env python3
  
import rospy
import cv2
import os,rospkg
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import Path,Odometry
from morai_msgs.msg import GPSMessage
from cv_bridge import CvBridgeError
from cv_bridge import CvBridge 
from slidewindow import SlideWindow
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import Float64,Int16,Float32MultiArray
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from pyproj import Proj

class IMGParser2:
    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.traffic_pub= rospy.Publisher('/camera_traffic', CtrlCmd, queue_size=1)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.ctrl_msg = CtrlCmd()
        self.x, self.y = None, None
        self.is_imu = False
        self.gps_imu = False
        self.proj_UTM = Proj(proj='utm',zone=52,ellps='WGS84', preserve_units = False)
        self.num=0
        self.crop_pts = np.array(
            [[
            [300,220],
            [300,170],
            [410,170],
            [410,220] 
            ]]
        )
        self.odom_msg = Odometry()
        rospy.spin()

    def callback(self, msg):
        if (self.odom_msg.pose.pose.position.x>=402456.5 and self.odom_msg.pose.pose.position.x<=402466.2 and self.odom_msg.pose.pose.position.y>=4133026 and self.odom_msg.pose.pose.position.y<=4133029) or (self.odom_msg.pose.pose.position.x>=402412.6 and self.odom_msg.pose.pose.position.x<=402429 and self.odom_msg.pose.pose.position.y>=4133027 and self.odom_msg.pose.pose.position.y<=4133032) or (self.odom_msg.pose.pose.position.x>=402442 and self.odom_msg.pose.pose.position.x<=402448 and self.odom_msg.pose.pose.position.y>=4133043.5 and self.odom_msg.pose.pose.position.y<=4133053.2):
            try:
                np_arr = np.fromstring(msg.data, np.uint8)
                img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            except CvBridgeError as e:
                print(e)
            
            self.mask = self.mask_roi(img_bgr)
            self.warped_img = self.warp_image(img_bgr)
            self.warped_img2 = self.warp_image(img_bgr)
            #cv2.imshow("Imw",self.warped_img)
            
            hsv = cv2.cvtColor(self.warped_img, cv2.COLOR_BGR2HSV)

            lower_red = np.array([170, 100, 100])
            upper_red = np.array([180, 255, 255])

            lower_green = np.array([50, 100, 100])
            upper_green = np.array([80, 255, 255])



            mask1 = cv2.inRange(hsv, lower_red, upper_red)
            mask3 = cv2.inRange(hsv, lower_green, upper_green)


            res = cv2.bitwise_and(self.warped_img2,self.warped_img2, mask=mask1)
            res2 = cv2.bitwise_and(self.warped_img2,self.warped_img2, mask=mask3)

            #빨간불, 노란불
            img = cv2.medianBlur(res, 5)
            ccimg = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
            cimg = cv2.cvtColor(ccimg, cv2.COLOR_BGR2GRAY)
            circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1, 30, param1=60, param2=10, minRadius=0, maxRadius=10)
            if circles is not None:
                self.num=1
                print('멈춰')

            #초록불
            img2 = cv2.medianBlur(res2, 5)
            ccimg2 = cv2.cvtColor(img2, cv2.COLOR_HSV2BGR)
            cimg2 = cv2.cvtColor(ccimg2, cv2.COLOR_BGR2GRAY)
            circles2 = cv2.HoughCircles(cimg2, cv2.HOUGH_GRADIENT, 1, 30, param1=60, param2=10, minRadius=0, maxRadius=10)
            if circles2 is not None:
                self.num=0
                print('가자')
            
            
            self.traffic_callback(self.num)
            
            cv2.imshow('res', res)
            cv2.imshow('res2', res2)
            cv2.imshow("Image window",self.mask)
            cv2.waitKey(1)
        else:
            cv2.destroyAllWindows()

    
    def mask_roi(self, img):
        h = img.shape[0]
        w = img.shape[1]

        if len(img.shape)==3:
            c = img.shape[2]
            mask = np.zeros((h,w,c),dtype=np.uint8)

            #mask_value = (0,10,255) # b,g,r
            mask_value=(255,255,255)
        else:

            mask = np.zeros((h,w),dtype=np.uint8)
            mask_value = (255)
        
        cv2.fillPoly(mask, self.crop_pts, mask_value)

        mask=cv2.bitwise_and(mask, img)

        return mask

    def warp_image(self,img):
        image_size = (img.shape[1],img.shape[0])

        H = 480 #480
        W = 640#640
        
        #source_points= np.float32([[50, 350], [640, 350], [50, 255], [640, 255]])#좌하, 우하, 좌상, 우상
        source_points = np.float32([[300,220],
            [410,220],
            [300,170],
            [410,170]
             ])
        destination_points = np.float32([[0, H], [W, H], [0, 0], [W, 0]])

        perspective_transform = cv2.getPerspectiveTransform(source_points,destination_points)
        warped_img = cv2.warpPerspective(img,perspective_transform, image_size, flags=cv2.INTER_LINEAR)
        
        return warped_img
    
    def traffic_callback(self,num) :
        if num==1:
            self.ctrl_msg.brake = 1
        else:
            self.ctrl_msg.brake = 0
            
        self.traffic_pub.publish(self.ctrl_msg)

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
        
    
    def convertLL2UTM(self):
        
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x = xy_zone[0]
        self.y = xy_zone[1]
            

if __name__ == '__main__':
    rospy.init_node('image_parser2',anonymous=True)

    image_parser = IMGParser2()

    rospy.spin()