#!/usr/bin/python

import rospy
import sys
import os
from sensor_msgs.msg import PointCloud


file_pcl = "/mnt/WD500/Espeleo/JINT/cave_pointcloud_depth_5.pcd"
num_pcl = 0


def callback(data):
    
    if subscribe_data:
    
        global xyz_pcl, rgb_pcl, num_pcl
        
        xyz_pcl = data.points
        rgb_pcl = data.channels[0].values
        num_pcl = len(data.points)
        
        sys.stdout.write(str(num_pcl) + " Points Subscribed...\n")
        sys.stdout.flush()


def main_function():
    
    global subscribe_data
    
    if os.path.exists(file_pcl):
        os.remove(file_pcl)
    
    rospy.init_node("cloud_to_file", anonymous=True)
    rospy.Subscriber("/cloud/points", PointCloud, callback)
    
    sys.stdout.write("Saving PointCloud in " + file_pcl + "\n")
    sys.stdout.flush()
    
    subscribe_data = True
    while not rospy.is_shutdown(): pass
    subscribe_data = False
        
    if raw_input("\nSave Pointcloud file? (y/n): ") == "y" and num_pcl > 0:
    
        f = open(file_pcl,"w")
        
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z rgb\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write("WIDTH 1\n")
        f.write("HEIGHT %s\n" % num_pcl)
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS %s\n" % num_pcl)
        f.write("DATA ascii")
        
        k = 0
        while k < num_pcl:
            
            f.write("\n%s %s %s %s" % (xyz_pcl[k].x, xyz_pcl[k].y, xyz_pcl[k].z, rgb_pcl[k]))
            k += 1
            sys.stdout.write("\rSaving PointCloud File... %5.1f%%" % (100.0*k/num_pcl))
            sys.stdout.flush()
            
            rospy.sleep(0.0001)
            
        sys.stdout.write("\n")
        sys.stdout.flush()
        f.close()


if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass