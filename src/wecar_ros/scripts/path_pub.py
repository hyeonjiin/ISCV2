#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from nav_msgs.msg import Path
from lib.utils import pathReader


class path :

    def __init__(self):
        rospy.init_node('wecar_planner', anonymous=True)
        
        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]
        self.path_msg = Path()
        path_reader=pathReader('wecar_ros')
        
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1)
        
        self.global_path=path_reader.read_txt(self.path_name+".txt")
        
        #self.global_path 자체가 nav_msgs/Path 타입을 가지고 있다.
        #그래서 바로 publish해주었다!
        #nav_msgs/path는 global_path.poses[1].pose.position 이런식으로 접근해야 한다.
        #그래서 pure_pursuit.py에서 for문을 돌릴 때 이 부분을 확인할 수 있다.
        
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            local_path_pub.publish(self.global_path)
        rate.sleep()

if __name__ == '__main__':
    try:
        path_run = path()
    except rospy.ROSInterruptException:
        pass

