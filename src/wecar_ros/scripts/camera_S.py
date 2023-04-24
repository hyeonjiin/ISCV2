#!/usr/bin/env python3
  
import rospy
import cv2
import os,rospkg
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from morai_msgs.msg import GPSMessage
from cv_bridge import CvBridgeError
from cv_bridge import CvBridge 
from slidewindow import SlideWindow
from morai_msgs.msg import CtrlCmd
import tf
from std_msgs.msg import Float64,Int16,Float32MultiArray

class IMGParser3:
    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.initialized = False
        self.bridge = CvBridge()
        self.slidewindow = SlideWindow()
        self.camera_pub= rospy.Publisher('/camera_steering', CtrlCmd, queue_size=1)
        self.ctrl_msg = CtrlCmd()
        self.steering_angle = 0.0
        self.last_steering=0.0
        # self.crop_pts = np.array(
        #     [[
        #     [0,420],
        #     [210,310],
        #     [490,310],
        #     [640,400]
        #     ]]
        # )
        rospy.spin()
    def callback(self, msg):
        if self.lat == 0 and self.lon ==0:
            try:
                img_bgr = self.bridge.compressed_imgmsg_to_cv2(msg)
            except CvBridgeError as e:
                print(e)

            img_warp = self.warp_image(img_bgr)

            # if self.initialized == False:
            #     cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL) 
            #     cv2.createTrackbar('low_H', 'Simulator_Image', 50, 255, self.nothing)
            #     cv2.createTrackbar('low_S', 'Simulator_Image', 50, 255, self.nothing)
            #     cv2.createTrackbar('low_V', 'Simulator_Image', 50, 255, self.nothing)
            #     cv2.createTrackbar('high_H', 'Simulator_Image', 255, 255, self.nothing)
            #     cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, self.nothing)
            #     cv2.createTrackbar('high_V', 'Simulator_Image', 255, 255, self.nothing)
            #     self.initialized = True

            # cv2.imshow("Image window",img_warp)

            # low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
            # low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
            # low_V = cv2.getTrackbarPos('low_V', 'Simulator_Image')
            # high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
            # high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')
            # high_V = cv2.getTrackbarPos('high_V', 'Simulator_Image')


            # cv2.cvtColor(img_warp, cv2.COLOR_BGR2HSV) # BGR to HSV
            #흰색
            lower_lane = np.array([142,149,156]) 
            upper_lane = np.array([188,184,208])
            #노란색
            # lower_lane = np.array([0,134,214]) 
            # upper_lane = np.array([255,191,255])
            #회색에 가까움
            #lower_lane = np.array([145,154,160]) 
            #upper_lane = np.array([160,211,199])
            # lower_lane = np.array([low_H, low_S, low_V]) # 
            # upper_lane = np.array([high_H, high_S, high_V])

            lane_image = cv2.inRange(img_warp, lower_lane, upper_lane)
            mask = cv2.inRange(img_warp, lower_lane, upper_lane)
            pixel_count = cv2.countNonZero(mask)

            #lane_image = cv2.inRange(img_warp, lower_lane, upper_lane)

            cv2.imshow("Lane Image", lane_image)
            print(pixel_count)
            #self.lane_detection(lane_image)

            cv2.waitKey(1)
    
    def warp_image(self,img):
        image_size = (img.shape[1],img.shape[0])

        H = 480 #480
        W = 640#640
        source_points= np.float32([[210, 350], [270, 350], [240, 340], [270, 340]])#좌하, 우하, 좌상, 우상
        destination_points = np.float32([[50, H-100], [W, H-100], [50, 0], [W, 0]])

        perspective_transform = cv2.getPerspectiveTransform(source_points,destination_points)
        warped_img = cv2.warpPerspective(img,perspective_transform, image_size, flags=cv2.INTER_LINEAR)
        
        return warped_img
    
    def nothing(self,x):
        pass
    
    def navsat_callback(self, gps_msg):
        self.gps_imu = True
        
        self.latitude = gps_msg.latitude
        self.longitude = gps_msg.longitude
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
      

if __name__ == '__main__':
    rospy.init_node('image_parser',anonymous=True)

    image_parser = IMGParser3()

    rospy.spin()