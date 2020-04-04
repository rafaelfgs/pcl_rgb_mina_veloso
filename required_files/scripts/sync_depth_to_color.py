#!/usr/bin/env python

import rospy
import sys
from copy import copy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

freq = 0.5
dmin = 1000
dmax = 2500

msg_depth_raw = Image()
msg_depth_info = CameraInfo()
msg_color_raw = Image()
msg_color_info = CameraInfo()
bridge = CvBridge()

def callback_depth_raw(data):
    global img_depth_raw, msg_depth_raw
    img_depth_raw = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    img_depth_raw.setflags(write=1)
    img_depth_raw[(img_depth_raw < dmin) | (img_depth_raw > dmax)] = 0
    msg_depth_raw = bridge.cv2_to_imgmsg(img_depth_raw, encoding="passthrough")
    msg_depth_raw.header.seq = copy(data.header.seq)
    msg_depth_raw.header.stamp = copy(data.header.stamp)
    msg_depth_raw.header.frame_id = "d435i_depth_optical_frame"

def callback_depth_info(data):
    global msg_depth_info
    msg_depth_info = copy(data)
    msg_depth_info.header.frame_id = "d435i_depth_optical_frame"

def callback_color_raw(data):
    global msg_color_raw
    msg_color_raw = copy(data)
    msg_color_raw.header.frame_id = "d435i_depth_optical_frame"

def callback_color_info(data):
    global msg_color_info
    msg_color_info = copy(data)
    msg_color_info.header.frame_id = "d435i_depth_optical_frame"

def main_function():
    
    rospy.init_node("sync_node", anonymous=True)
    
    rospy.Subscriber("/d435i/depth/image_rect_raw", Image, callback_depth_raw)
    rospy.Subscriber("/d435i/depth/camera_info", CameraInfo, callback_depth_info)
    rospy.Subscriber("/d435i/color/image_raw", Image, callback_color_raw)
    rospy.Subscriber("/d435i/color/camera_info", CameraInfo, callback_color_info)
    
    pub_depth_raw = rospy.Publisher("/depth_registered/image_rect", Image, queue_size=10)
    pub_depth_info = rospy.Publisher("/depth_registered/camera_info", CameraInfo, queue_size=10)
    pub_color_raw = rospy.Publisher("/rgb/image_rect_color", Image, queue_size=10)
    pub_color_info = rospy.Publisher("/rgb/camera_info", CameraInfo, queue_size=10)
    
    sys.stdout.write("\nSynchronizing Images Publishing...\n")
    
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        
        pub_depth_raw.publish(msg_depth_raw)
        pub_depth_info.publish(msg_depth_info)
        pub_color_raw.publish(msg_color_raw)
        pub_color_info.publish(msg_color_info)
        
        rate.sleep()

if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass