#!/usr/bin/env python
# filter.py
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
import sys; sys.path.append('/intel/euclid/euclid_ws/devel/lib/python2.7/dist-packages/rgb_filter/')
from cfg import RGBFilterConfig
#from rgb_filter.cfg import RGBFilterConfig

bridge = CvBridge()

color_b_lower = 0
color_b_upper = 255
color_g_lower = 0
color_g_upper = 255
color_r_lower = 0
color_r_upper = 255

def configCallback(config, level):
 global color_b_lower, color_b_upper
 global color_g_lower, color_g_upper
 global color_r_lower, color_r_upper

 color_b_lower = int(config["color_b_lower"])
 color_b_upper = int(config["color_b_upper"])
 color_g_lower = int(config["color_g_lower"])
 color_g_upper = int(config["color_g_upper"])
 color_r_lower = int(config["color_r_lower"])
 color_r_upper = int(config["color_r_upper"])


 return config

def imgCallback(data):
 global color_b_lower, color_b_upper
 global color_g_lower, color_g_upper
 global color_r_lower, color_r_upper
 global pub

 img = bridge.imgmsg_to_cv2(data,"bgr8")

 lower_bound = np.array([color_b_lower, color_g_lower, color_r_lower])
 upper_bound = np.array([color_b_upper, color_g_upper, color_r_upper])

 mask = cv2.inRange(img,lower_bound,upper_bound)
 res = cv2.bitwise_and(img,img,mask=mask)
 ros_img = bridge.cv2_to_imgmsg(res,"bgr8")
 pub.publish(ros_img)

def rgb_filter():
 global pub

 rospy.init_node('rgb_filter', anonymous=True)
 pub = rospy.Publisher('/camera/color/filtered', Image, queue_size=10)
 rospy.Subscriber("/camera/color/image_raw", Image, imgCallback)
 srv = Server(RGBFilterConfig, configCallback)
 rospy.spin()

if __name__ == '__main__':
 rgb_filter()
