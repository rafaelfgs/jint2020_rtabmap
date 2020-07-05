#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from bib_espeleo_differential import espeleo_differential

motor_publisher1 = rospy.Publisher('/device1/set_joint_state', JointState, queue_size=10)
motor_publisher2 = rospy.Publisher('/device2/set_joint_state', JointState, queue_size=10)
motor_publisher3 = rospy.Publisher('/device3/set_joint_state', JointState, queue_size=10)
motor_publisher4 = rospy.Publisher('/device4/set_joint_state', JointState, queue_size=10)
motor_publisher5 = rospy.Publisher('/device5/set_joint_state', JointState, queue_size=10)
motor_publisher6 = rospy.Publisher('/device6/set_joint_state', JointState, queue_size=10)

last_time = rospy.Time()

wheel_velocity = [0, 0, 0, 0, 0, 0]

forward_orientation = True
turbo_button = True

def dyn_config_callback(cfg):
    global forward_orientation
    global turbo_button

    if "forward_orientation" in cfg.keys():
        forward_orientation = cfg["forward_orientation"]
        print(cfg)

    if "turbo_button" in cfg.keys():
        turbo_button = cfg["turbo_button"]
	

def callback(data):
    global last_time, wheel_velocity, forward_orientation

    cmd_vel_x = data.linear.x

    #if (turbo_button) == True:
    #    if (data.linear.x >= 0.4):
    #        cmd_vel_x = 0.4
    #    elif(data.linear.x <= -0.4):
    #        cmd_vel_x = -0.4

    if forward_orientation:
	pass
    else:
        cmd_vel_x = cmd_vel_x * (-1)
	
	
    w = data.angular.z

    # Correction for cmd-vel - max the motor rpm to 9700 when v_linear = 1
    v = cmd_vel_x * espeleo.cmd_vel_correction

    wheel_velocity = espeleo.set_espeleo_velocity(v, w)
    last_time = rospy.Time.now()
    

def listener():
    global last_time, wheel_velocity

    rospy.init_node('espeleo_locomotion_wheels', anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber("cmd_vel", Twist, callback)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():

        if ((rospy.get_rostime() - last_time).to_sec() > 2):
            stop_state = JointState()
            stop_state.header.stamp = rospy.Time.now()
            stop_state.velocity = [0]

            motor_publisher1.publish(stop_state)
            motor_publisher2.publish(stop_state)
            motor_publisher3.publish(stop_state)
            motor_publisher4.publish(stop_state)
            motor_publisher5.publish(stop_state)
            motor_publisher6.publish(stop_state)

        else:

            state_right = JointState()
            state_right.header.stamp = rospy.Time.now()
            state_right.velocity = [wheel_velocity[3]]
            
            state_right_middle = JointState()
            state_right_middle.header.stamp = rospy.Time.now()
            state_right_middle.velocity = [wheel_velocity[4]]
            
            state_left = JointState()
            state_left.header.stamp = rospy.Time.now()
            state_left.velocity = [wheel_velocity[0]]
            
            state_left_middle = JointState()
            state_left_middle.header.stamp = rospy.Time.now()
            state_left_middle.velocity = [wheel_velocity[1]]
            
            motor_publisher1.publish(state_left)
            motor_publisher2.publish(state_left_middle)
            motor_publisher3.publish(state_left)
            motor_publisher4.publish(state_right)
            motor_publisher5.publish(state_right_middle)
            motor_publisher6.publish(state_right)

        r.sleep()


if __name__ == '__main__':
    client = dynamic_reconfigure.client.Client("espeleo", timeout=5, config_callback=dyn_config_callback)
    espeleo = espeleo_differential()
    listener()
