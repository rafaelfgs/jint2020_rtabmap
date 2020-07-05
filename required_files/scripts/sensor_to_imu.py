#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point

is_pub = [False, False]

def callback_acc(data):
    global acc_msg, is_pub
    acc_msg = [data.x, data.y, data.z]
    is_pub[0] = True

def callback_gyr(data):
    global gyr_msg, is_pub
    gyr_msg = [data.x, data.y, data.z]
    is_pub[1] = True

def main_function():
    
    global is_pub
    
    rospy.init_node("sensor_to_imu", anonymous=True)
    
    rospy.Subscriber("/sensors/acc", Point, callback_acc)
    rospy.Subscriber("/sensors/gyro", Point, callback_gyr)
    pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=10)
    
    rate = rospy.Rate(1000)
    
    while not all(is_pub) and not rospy.is_shutdown():
        rate.sleep()
    
    sys.stdout.write("Publishing Merged IMU...\n")
    sys.stdout.flush()
    
    while not rospy.is_shutdown():
        
        if all(is_pub):
            is_pub[0] = False
            is_pub[1] = False
            msg = Imu()
            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "imu_link"
            msg.angular_velocity.x = gyr_msg[0]
            msg.angular_velocity.y = gyr_msg[1]
            msg.angular_velocity.z = gyr_msg[2]
            msg.linear_acceleration.x = acc_msg[0]
            msg.linear_acceleration.y = acc_msg[1]
            msg.linear_acceleration.z = acc_msg[2]
            pub.publish(msg)
            
        rate.sleep()

if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
