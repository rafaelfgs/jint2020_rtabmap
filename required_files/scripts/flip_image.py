#!/usr/bin/env python
import rospy
import sys
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image



bridge = CvBridge()
msg_bool = [False, False]



def callback_rgb(data_rgb):
    
    global msg_bool, flip_rgb, rgb
    flip_rgb = bridge.imgmsg_to_cv2(data_rgb, desired_encoding="rgb8")
    rgb = cv2.flip(flip_rgb,0)
    msg_rgb = bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
    msg_rgb.header = data_rgb.header
    pub_rgb = rospy.Publisher("/rgb/image_rect", Image, queue_size=10)
    pub_rgb.publish(msg_rgb)
    msg_bool[0] = True



def callback_depth(data_depth):
    
    global msg_bool, flip_depth, depth
    flip_depth = bridge.imgmsg_to_cv2(data_depth, desired_encoding="16UC1")
    depth = cv2.flip(flip_depth,0)
    msg_depth = bridge.cv2_to_imgmsg(depth, encoding="16UC1")
    msg_depth.header = data_depth.header
    pub_depth = rospy.Publisher("/depth/image_rect", Image, queue_size=10)
    pub_depth.publish(msg_depth)
    msg_bool[1] = True



def main_function():
    
    rospy.init_node("flip_image", anonymous=True)
    rospy.Subscriber("/rgb/image_rect_flipped", Image, callback_rgb)
    rospy.Subscriber("/depth/image_rect_flipped", Image, callback_depth)
    rate = rospy.Rate(10)
    
    while not all(msg_bool) and not rospy.is_shutdown():
        rate.sleep()
    
    sys.stdout.write("Publishing Flipped Images...\n")
    sys.stdout.flush()
    
    while not rospy.is_shutdown():
        rate.sleep()



if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass