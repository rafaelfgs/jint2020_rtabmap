#!/usr/bin/python2.7
import rospy
import yaml
import os

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from ros_eposmcd_msgs.msg import Movement
from ros_eposmcd_msgs.msg import MovementArray
from ros_eposmcd_msgs.msg import EspeleoJoints
from sensor_msgs.msg import JointState

from bib_locomotion import bib_espeleo_locomotion 

class espeleo_locomotion():

    # constructor class
    def __init__(self):
        
        # initilializing node
        rospy.init_node('espeleo_locomotion', anonymous=True)

        # sending a message
        rospy.loginfo('Espeleo_Locomotion 2.0 node initiated.')

        # subscribing to cmd_vel
        rospy.Subscriber("cmd_vel", Twist, self.callback_command)
       
        # subscribing to joints positions
        rospy.Subscriber("ros_eposmcd/joints_positions", EspeleoJoints, self.callback_jointsPos)

        # creating the three command topics publisher
        self.pub_posAbs = rospy.Publisher("ros_eposmcd/position_movement_absolute", MovementArray, queue_size=1)
        self.pub_posRel = rospy.Publisher("ros_eposmcd/position_movement_relative", MovementArray, queue_size=1)
        self.pub_vel1   = rospy.Publisher('/device1/set_joint_state', JointState, queue_size=1)
        self.pub_vel2   = rospy.Publisher('/device2/set_joint_state', JointState, queue_size=1)
        self.pub_vel3   = rospy.Publisher('/device3/set_joint_state', JointState, queue_size=1)
        self.pub_vel4   = rospy.Publisher('/device4/set_joint_state', JointState, queue_size=1)
        self.pub_vel5   = rospy.Publisher('/device5/set_joint_state', JointState, queue_size=1)
        self.pub_vel6   = rospy.Publisher('/device6/set_joint_state', JointState, queue_size=1)

        # trying to get robot parameters from ROS parameter server
        try:
            locomotionParamDictionary = rospy.get_param('/espeleo_locomotion/')     
        except:
            
            # sends a fatal message to the user
            rospy.logfatal("Espeleo default parameters not found on ROS param server. Check if ./espeleo_locomotion/config/locomotion_parameters.yaml file exists, and if it is being correctly loaded into a launch file. Shutting down the node")

            # shuts down the node
            rospy.signal_shutdown('Shutting down the node.')
            return None

        # instantiating the locomotion library with the locomotion dictionary
        self.esp_loc = bib_espeleo_locomotion(locomotionParamDictionary)

        # defining the loop rate [Hz]cld
        rate = rospy.Rate(locomotionParamDictionary['control_rate']) 

        # infinite loop
        while not rospy.is_shutdown():

            # rospy.loginfo('Num entrance: %s. Ancient: %s and new %s. The flag is %s' % (auxi, self.espeleo_input_last['linear'], self.espeleo_input['linear'], flag_dif))

            # retrieves ROS time
            ros_time = rospy.Time.now()

            # sends the actual command to the library, obtaining the desired control
            flag_sendControl,command_list = self.esp_loc.control_update(ros_time)

            # tests if there is need for a command publish
            if flag_sendControl:

                # publishes the control command
                state = JointState()
                
                state.velocity = [command_list.movement_command[0].velocity]
                self.pub_vel1.publish(state)

                state.velocity = [command_list.movement_command[1].velocity]
                self.pub_vel2.publish(state)

                state.velocity = [command_list.movement_command[2].velocity]
                self.pub_vel3.publish(state)

                state.velocity = [command_list.movement_command[3].velocity]
                self.pub_vel4.publish(state)

                state.velocity = [command_list.movement_command[4].velocity]
                self.pub_vel5.publish(state)

                state.velocity = [command_list.movement_command[5].velocity]
                self.pub_vel6.publish(state)

            # sleeping
            rate.sleep()


    # --- Callback for the cmd_vel command
    def callback_command(self, data):

        # mounting the espelo input for the library
        espeleo_input = {'time_stamp':0, 'linear':data.linear.x, 'angular':data.angular.z}

        # setting the locomotion set point
        self.esp_loc.set_locomotionControlParameters(espeleo_input)   


    # --- Callback for the joints position receival
    def callback_jointsPos(self, data):

        # sets encoders values into the library        
        self.esp_loc.set_joints_position(data.jointsPositions)


#================= starts the node ========================
if __name__ == '__main__':

    espeleo_locomotion()

