#!/usr/bin/env python

"""
This node receives the motors telemetry from the espeleorobo and calculates the odometry based on the wheels velocity
It subscribes to the wheels velocities in ros_eposmcd/motor1 to motor6, published by ros_eposmcd
It publishes the odometry to odom topic
It can calculate the odometry using differential or skidsteering kinematic models,
just change the flag skid_steer to 1 if you want skid steer or to 0 if you want differential
The parameters used both for robot and skidsteer come from Eduardo Cota master thesis
https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
"""

from math import sin, cos, pi

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState


from bib_espeleo_differential import espeleo_differential

class odometry:
    def __init__(self):

        # Kinematic model
        self.skid_steer = 0

        # Robot Pose
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.Dth = 0.0

        # Motor velocities
        self.motor_velocity1 = 0
        self.motor_velocity2 = 0
        self.motor_velocity3 = 0
        self.motor_velocity4 = 0
        self.motor_velocity5 = 0
        self.motor_velocity6 = 0
        
        # Skidsteer parameters
        self.alpha_skid = rospy.get_param('espeleo_locomotion/wheeled_kinematic_lambda')
        self.ycir_skid = rospy.get_param('espeleo_locomotion/wheeled_kinematic_ycir')

        self.time_counter_aux = 0
        self.ros_init()

    def ros_init(self):

        rospy.init_node('wheel_odometry_publisher', anonymous=True)

        # Times used to integrate velocity to pose
        self.current_time = 0.0
        self.last_time = 0.0

        # Create subscribers that receives the wheels velocities
        self.subscriber_motor1 = rospy.Subscriber("/device1/get_joint_state", JointState, self.motor1_callback)
        self.subscriber_motor2 = rospy.Subscriber("/device2/get_joint_state", JointState, self.motor2_callback)
        self.subscriber_motor3 = rospy.Subscriber("/device3/get_joint_state", JointState, self.motor3_callback)
        self.subscriber_motor4 = rospy.Subscriber("/device4/get_joint_state", JointState, self.motor4_callback)
        self.subscriber_motor5 = rospy.Subscriber("/device5/get_joint_state", JointState, self.motor5_callback)
        self.subscriber_motor6 = rospy.Subscriber("/device6/get_joint_state", JointState, self.motor6_callback)

        # odom publisher
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.vel_pub = rospy.Publisher("robot_vel", Twist, queue_size=1)

        # Tf broadcaster
        self.odom_broadcaster = tf.TransformBroadcaster()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    # Motor velocity callbacks
    def motor1_callback(self, message):
        self.motor_velocity1 = message.velocity[0]

    def motor2_callback(self, message):
        self.motor_velocity2 = message.velocity[0]

    def motor3_callback(self, message):
        self.motor_velocity3 = message.velocity[0]

    def motor4_callback(self, message):
        self.motor_velocity4 = message.velocity[0]

    def motor5_callback(self, message):
        self.motor_velocity5 = message.velocity[0]

    def motor6_callback(self, message):
        self.motor_velocity6 = message.velocity[0]
        self.current_time = message.header.stamp.secs + message.header.stamp.nsecs*0.000000001
        self.odometry_calculation()

    def odometry_calculation(self):

        if self.skid_steer:
            ### Skid-Steering model

            velocity_left, velocity_right = espeleo.left_right_velocity([self.motor_velocity1,  self.motor_velocity2,  self.motor_velocity3,  self.motor_velocity4,  self.motor_velocity5,  self.motor_velocity6])

            # Linear velocity
            v_robot = (velocity_right + velocity_left) * (self.alpha_skid / 2)

            # Angular velocity
            w_robot = ((velocity_right - velocity_left) * (self.alpha_skid / (2 * self.ycir_skid)))

            if self.time_counter_aux == 0:
                self.last_time = self.current_time
                self.time_counter_aux = 1

            # Velocity in the XY plane
            vx_robot = v_robot * cos(self.th)
            vy_robot = v_robot * sin(self.th)

            # Calculating odometry
            dt = self.current_time - self.last_time
            delta_x = vx_robot * dt
            delta_y = vy_robot * dt
            delta_th = w_robot * dt

            # Integrating pose
            self.x += delta_x
            self.y += delta_y

            self.th += delta_th


        else:

            v_robot, w_robot = espeleo.get_espeleo_velocity([self.motor_velocity1,  self.motor_velocity2,  self.motor_velocity3,  self.motor_velocity4,  self.motor_velocity5,  self.motor_velocity6])
            
            # Velocity in the XY plane
            vx_robot = v_robot * cos(self.th)
            vy_robot = v_robot * sin(self.th)

            if self.time_counter_aux == 0:
                self.last_time = self.current_time
                self.time_counter_aux = 1

            # Calculating odometry
            dt = self.current_time - self.last_time
            delta_x = vx_robot * dt
            delta_y = vy_robot * dt
            delta_th = w_robot * dt

            # Integrating pose
            self.x += delta_x
            self.y += delta_y
            self.th += delta_th


        vel_robot = Twist()
        vel_robot.linear.x = v_robot
        vel_robot.angular.z = w_robot

        # Since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # First, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            rospy.Time.now(),
            "base_link",
            "odom"  # odom
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(v_robot, 0, 0), Vector3(0, 0, w_robot))

        # Calculating the covariance based on the angular velocity
        # if the robot is rotating, the covariance is higher than if its going straight
        if abs(w_robot) > 0.2:
            covariance_cons = 0.3
        else:
            covariance_cons = 0.05

        odom.pose.covariance[0] = covariance_cons
        odom.pose.covariance[7] = covariance_cons
        odom.pose.covariance[35] = 100 * covariance_cons

        odom.pose.covariance[1] = covariance_cons
        odom.pose.covariance[6] = covariance_cons

        odom.pose.covariance[31] = covariance_cons
        odom.pose.covariance[11] = covariance_cons

        odom.pose.covariance[30] = 10 * covariance_cons
        odom.pose.covariance[5] = 10 * covariance_cons

        odom.pose.covariance[14] = 0.1
        odom.pose.covariance[21] = 0.1
        odom.pose.covariance[28] = 0.1

        odom.twist.covariance[0] = covariance_cons
        odom.twist.covariance[7] = covariance_cons
        odom.twist.covariance[35] = 100 * covariance_cons

        odom.twist.covariance[1] = covariance_cons
        odom.twist.covariance[6] = covariance_cons

        odom.twist.covariance[31] = covariance_cons
        odom.twist.covariance[11] = covariance_cons

        odom.twist.covariance[30] = 10 * covariance_cons
        odom.twist.covariance[5] = 10 * covariance_cons

        odom.twist.covariance[14] = 0.1
        odom.twist.covariance[21] = 0.1
        odom.twist.covariance[28] = 0.1

        # publish the message
        self.odom_pub.publish(odom)
        self.vel_pub.publish(vel_robot)

        self.last_time = self.current_time


if __name__ == '__main__':
    espeleo = espeleo_differential()
    odometry_obj = odometry()
