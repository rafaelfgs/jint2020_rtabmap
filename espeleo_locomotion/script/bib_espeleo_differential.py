#!/usr/bin/env python

import rospy

class espeleo_differential:
    def __init__(self):

        # Wheel data
        self.wheel_radius = rospy.get_param('espeleo_locomotion/wheeled_radius')
        self.wheel_diameter = self.wheel_radius * 2
        self.number_wheels = rospy.get_param('espeleo_locomotion/number_wheels')
        self.internal_width = rospy.get_param('espeleo_locomotion/internal_wheel_width')
        self.external_width = rospy.get_param('espeleo_locomotion/external_wheel_width')

        # Reductions (Planetary gear reduction, internal and external synchronous belt wheel reduction)
        self.planetary_reduction = rospy.get_param('espeleo_locomotion/planetary_gear_reduction')
        self.central_reduction = rospy.get_param('espeleo_locomotion/central_wheel_reduction')
        self.extremity_reduction = rospy.get_param('espeleo_locomotion/extremity_wheel_reduction')

        # Constants
        self.rad_const = (0.10471975511965977)  #(2*pi)/60

        # Motor velocities
        self.motor_velocity2 = 0
        self.motor_velocity3 = 0
        self.motor_velocity4 = 0
        self.motor_velocity5 = 0
        self.motor_velocity6 = 0
        self.cmd_vel_correction = self.wheel_radius * 4.36762


    # Get a list or float of RPM values and convert then to rad/s
    def rpm_to_rad(self, velocities):
        if isinstance(velocities, list):
            velocities = [self.rad_const * x for x in velocities]
        else:
            velocities = self.rad_const * velocities
        return (velocities)


    # Get a list or float of rad/s and convert then to RPM
    def rad_to_rpm(self, velocities):
        if isinstance(velocities, list):
            velocities = [(1/self.rad_const) * x for x in velocities]
        else:
            velocities = (1/self.rad_const) * velocities
        return (velocities)


    # Transform RPM from external wheels motors to wheel's RPM
    def motor_to_extremity_wheels(self, motor_rpm):
        external_wheel = motor_rpm / (self.planetary_reduction * self.extremity_reduction)
        return external_wheel


    # Transform RPM from internal wheels motors to wheel's RPM
    def motor_to_central_wheels(self, motor_rpm):
        internal_wheel = motor_rpm / (self.planetary_reduction * self.central_reduction)
        return internal_wheel


    # Transform Wheel RPM to motor RPM
    def extremity_wheels_to_motor(self, wheel_rpm):
        motor_rpm = wheel_rpm * (self.planetary_reduction * self.extremity_reduction)
        return motor_rpm


    # Transform Wheel RPM to motor RPM
    def central_wheels_to_motor(self, wheel_rpm):
        motor_rpm = wheel_rpm * (self.planetary_reduction * self.central_reduction)
        return motor_rpm

    # Takes motor velocities and transforms then into wheel velocity, according to their position (internal external or)
    def wheel_velocity(self, motor_velocities):
        wheel_1 = self.motor_to_extremity_wheels(motor_velocities[0])
        wheel_2 = self.motor_to_central_wheels(motor_velocities[1])
        wheel_3 = self.motor_to_extremity_wheels(motor_velocities[2])
        wheel_4 = self.motor_to_extremity_wheels(motor_velocities[3])
        wheel_5 = self.motor_to_central_wheels(motor_velocities[4])
        wheel_6 = self.motor_to_extremity_wheels(motor_velocities[5])
        wheels = [wheel_1, wheel_2, wheel_3, -wheel_4, -wheel_5, -wheel_6]
        return wheels

# Receives an array of motor velocities and return Espeleo's Linear and Angular velocity

    def left_right_velocity(self, motor_velocities):

        # Takes motor velocities and transforms then into wheel velocity, according to their position (internal or external)
        wheel_1 = self.motor_to_extremity_wheels(motor_velocities[0])
        wheel_2 = self.motor_to_central_wheels(motor_velocities[1])
        wheel_3 = self.motor_to_extremity_wheels(motor_velocities[2])
        wheel_4 = self.motor_to_extremity_wheels(motor_velocities[3])
        wheel_5 = self.motor_to_central_wheels(motor_velocities[4])
        wheel_6 = self.motor_to_extremity_wheels(motor_velocities[5])
        wheels = [wheel_1, wheel_2, wheel_3, wheel_4, wheel_5, wheel_6]

        # Transforms RPM to rad/s
        wheels = self.rpm_to_rad(wheels)

        # Acording to wheels configuration, calculate the linear and angular velocity
        if self.number_wheels == 4:
            velocity_left = [wheels[0], wheels[2]]
            velocity_right = [wheels[3], wheels[5]]

        else:
            velocity_left = [wheels[0], wheels[1], wheels[2]]
            velocity_right = [wheels[3], wheels[4], wheels[5]]

        # Calculate right and left velocitys, considering that when the Espeleo move fowards, the motor on the left has a positive velocity, and the motors on the right has negative.
        velocity_left = sum(velocity_left) / len(velocity_left)
        velocity_right = (sum(velocity_right) / len(velocity_right)) * -1
        return (velocity_right, velocity_left)

    def get_espeleo_velocity(self, motor_velocities):


        velocity_right, velocity_left = self.left_right_velocity(motor_velocities)

        if self.number_wheels == 4:
            L = self.internal_width
        else:
            L = self.external_width

        v_espeleo = self.wheel_radius * (velocity_right + velocity_left) / 2
        w_espeleo = self.wheel_radius * (velocity_right - velocity_left) / L

        return v_espeleo, w_espeleo


    def set_espeleo_velocity(self, v_espeleo, w_espeleo ):

        # Get espeleo width param acording to number of wheels
        if self.number_wheels == 4:
            L = self.internal_width
        elif self.number_wheels == 6:
            L = self.external_width

        # Convert Linear ang Angular velocity to right and left velocity
        velocity_right = ((2 * v_espeleo) + (w_espeleo * L)) / (2 * self.wheel_radius)
        velocity_left = ((2 * v_espeleo) - (w_espeleo * L)) / (2 * self.wheel_radius)

        # Convert rad/s to rpm
        right_wheel_rpm = self.rad_to_rpm(velocity_right)
        left_wheel_rpm = self.rad_to_rpm(velocity_left)

        # Set max rpm to 12000
        if (right_wheel_rpm > 12000):
            right_wheel_rpm = 12000
        if (right_wheel_rpm < -12000):
            right_wheel_rpm = -12000
            
        if (left_wheel_rpm > 12000): 
            left_wheel_rpm = 12000
        if (left_wheel_rpm < -12000):
            left_wheel_rpm = -12000

        wheel_1 = self.extremity_wheels_to_motor(left_wheel_rpm)
        wheel_2 = self.central_wheels_to_motor(left_wheel_rpm)
        wheel_3 = self.extremity_wheels_to_motor(left_wheel_rpm)
        wheel_4 = self.extremity_wheels_to_motor(right_wheel_rpm)
        wheel_5 = self.central_wheels_to_motor(right_wheel_rpm)
        wheel_6 = self.extremity_wheels_to_motor(right_wheel_rpm)

        if self.number_wheels == 4:
            wheels = [wheel_1, 0, wheel_3, -wheel_4, 0, -wheel_6]
        elif self.number_wheels == 6:
            wheels = [wheel_1, wheel_2, wheel_3, -wheel_4, -wheel_5, -wheel_6]

        return wheels

