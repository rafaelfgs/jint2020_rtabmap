#!/usr/bin/env python

import rospy
import rosbag
import numpy
import sys
import os
#import cv2
#from math import pi
from copy import copy
from cv_bridge import CvBridge
#from nav_msgs.msg import Odometry
#from rosgraph_msgs.msg import Clock
#from geometry_msgs.msg import Pose
#from tf2_msgs.msg import TFMessage
#from geometry_msgs.msg import TransformStamped
#from tf.transformations import quaternion_from_euler


match_bags = False

path_file = "/media/rafael/Seagate Expansion Drive/2020_06_COPPELIA/3/"
input_file = sorted(os.listdir(path_file + "splitted/"))
output_file = [f for f in input_file if "_0." in f][0][:19] + "_rtab.bag"


def main_function():
    
    global msg
    
    rospy.init_node("correct_bag", anonymous=True)
    
    rospy.sleep(1.0)
    for k in range(len(input_file)):
        sys.stdout.write("\n%s" % (input_file[k]))
        sys.stdout.flush()
        
    rospy.sleep(0.2)
    if raw_input("\nAre you sure these are the correct files? (y/n): ") == "y":
        sys.stdout.write("\n%s\n" % (path_file + output_file))
        sys.stdout.flush()
        if os.path.exists(path_file + output_file):
            if raw_input("Output file already exists, do you want to replace it? (y/n): ") == "y":
                os.remove(path_file + output_file)
            else:
                sys.exit(0)
    else:
        sys.exit(0)
    
    bag_in = numpy.empty(len(input_file), dtype=object)
    bag_time = numpy.zeros((2,len(input_file)))
    
    k = 0
    
    while k < len(input_file) and not rospy.is_shutdown():
        
        sys.stdout.write("\nOpening bag %d... " % (k+1))
        sys.stdout.flush()
        bag_in[k] = rosbag.Bag(path_file + "splitted/" + input_file[k])
        sys.stdout.write("Ok!")
        sys.stdout.flush()
        
        bag_time[1,k] = bag_in[k].get_end_time() - bag_in[k].get_start_time()
        for topic, msg, t in bag_in[k].read_messages():
            bag_time[0,k] = rospy.Time.to_sec(t)
            break
        
        k += 1
    
    if match_bags:
        bag_start_time = min(bag_time[0])
        bag_end_time = min(bag_time[0]) + max(bag_time[1])
        bag_shift_time = bag_time[0] - min(bag_time[0])
    else:
        bag_start_time = min(bag_time[0])
        bag_end_time = max(bag_time[0] + bag_time[1])
        bag_shift_time = numpy.zeros(len(input_file))
    
    sys.stdout.write("\n\nBag starting: %10.9f\nBag ending:   %10.9f\nBag duration: %3.1fs\n" % (bag_start_time, bag_end_time, (bag_end_time-bag_start_time)))
    for k in range(len(input_file)):
        sys.stdout.write("Bag %d shift:  %3.1f\n" % ((k+1), bag_shift_time[k]))
    sys.stdout.flush()
    sys.stdout.write("\n")
    
    bag_out = rosbag.Bag(path_file + output_file, "w")
    
    bridge = CvBridge()
    k = 0
    
    while k < len(input_file):
        
        for topic, msg, t in bag_in[k].read_messages():
            
            topic_bool = False
            
            if rospy.is_shutdown():
                k = len(input_file)
                break
            
            if match_bags and hasattr(msg, "header"):
                t = rospy.Time.from_sec(rospy.Time.to_sec(t) - bag_shift_time[k])
                msg.header.stamp = copy(t)
            
            if False:
                pass
                
            elif topic == "/depth/camera_info":
                bag_out.write(topic, msg, t)
                topic_bool = True
                
            elif topic == "/depth/image_rect":
                bag_out.write(topic, msg, t)
                topic_bool = True
                
            elif topic == "/rgb/camera_info":
                bag_out.write(topic, msg, t)
                topic_bool = True
                
            elif topic == "/rgb/image_rect":
                bag_out.write(topic, msg, t)
                topic_bool = True
                
            elif topic == "/tf":
                bag_out.write(topic, msg, t)
                topic_bool = True
                
            if topic_bool:
                status_time = 100.0 * (rospy.Time.to_sec(t) - bag_start_time) / (bag_end_time - bag_start_time)
                sys.stdout.write("\rBag %2d/%2d - Publishing %-28s %5.1f%%" % ((k+1), len(input_file), topic, status_time))
                sys.stdout.flush()
        
        rospy.sleep(0.0001)
        sys.stdout.write("\n")
        sys.stdout.flush()
        
        k += 1
    
    for k in range(len(input_file)):
        bag_in[k].close()
    bag_out.close()
    
    sys.stdout.write("\nFiles Closed\n\n")
    sys.stdout.flush()


if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass