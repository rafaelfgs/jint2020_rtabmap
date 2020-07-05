import numpy as np
from ros_eposmcd_msgs.msg import Movement
from ros_eposmcd_msgs.msg import MovementArray

# Class definition
class bib_espeleo_locomotion:
    
    # class constructor
    def __init__(self, locomotionParamDictionary):

        # ---- robot parameters -----

        # --- saving the locomotion dictionary into a local variable
        # joints_MaxJointsSpeed: maximum EspeleoRobo joints speed
        # wheeled_radius: wheels radius in [m]
        # espeleo_maxTranslationalSpeed: maximum translational speed of Espeleo in [m/s]
        # espeleo_maxrotationalSpeed: maximum rotational speed in [rad/s] 
        # wheeled_kinematic_lambda and wheeled_kinematic_ycir  (see Filipe Rocha dissertation for + info)
        self.locomotionParamDictionary = locomotionParamDictionary

        # ---- initializing  usefull variables ----

        # joints encoder values
        self.joints_position = [None] * 6        # a vector for joints positions

        # locomotion set points
        self.locomotionControlDictionary = {}
        self.locomotionControlDictionary['linearSetPoint'] = 0
        self.locomotionControlDictionary['angularSetPoint'] = 0
        self.locomotionControlDictionary['lastValidTime'] = 0

        # --- computing locomotion modes persistent variables

        # computes wheeled mode kinematic A matrix (see, again, Filipe Rocha dissertation for + info)
        self.wheeled_kinematic_A = self.compute_kinematicAMatrix(self.locomotionParamDictionary['wheeled_kinematic_lambda'], self.locomotionParamDictionary['wheeled_radius'], self.locomotionParamDictionary['wheeled_kinematic_ycir'])


    # --- Method to update the robot joints control
    def control_update(self, rosmaster_time):


        # uses the right function for each locomotion mode
        if self.locomotionParamDictionary['locomotion_mode'] == 1: # for the wheeled modes

            # calls the trajectory generator
            command_list = self.control_update_sixWheels(rosmaster_time)

        elif self.locomotionParamDictionary['locomotion_mode'] == 2: # for the legged mode

            # calls the trajectory generator
            command_list = self.control_update_sixLegs(rosmaster_time)
        
        # returns appliable command for the motors
        else:
            command_list = -1
        
        # treats the sending command flag
        if command_list == -1:
            flag_sendCommand = False
        else:
            flag_sendCommand = True

        # returns the desired command 
        # the TRUE should be a flag indicating if a control is needed now
        return flag_sendCommand, command_list

    # --- generates joints commands values for the wheeled locomotion mode ---
    def control_update_sixWheels(self, rosmaster_time):

        # the returning list
        command_message_list = MovementArray()

        command_message_array = [Movement()]*6

        # computing desired speeds
        # Remark that set points value are from -1 to 1
        vel_linear_x = self.locomotionParamDictionary['espeleo_maxTranslationalSpeed'] * self.locomotionControlDictionary['linearSetPoint']
        vel_angular_z = self.locomotionParamDictionary['espeleo_maxRotationalSpeed'] * self.locomotionControlDictionary['angularSetPoint']

        # b matrix. The set points value must be inverted
        b = np.array([[vel_linear_x],[vel_angular_z]])

        # Solves the linear equation to find the joints control
        x = np.linalg.lstsq(self.wheeled_kinematic_A, b, rcond=None)[0]

        # acquiring the sides velocities
        omega_right = np.deg2rad(x[0][0])
        omega_left = np.deg2rad(x[1][0])

        # assembles the returning velocity list for all motors
        for i in range(0,6):

            msg = Movement()
            
            msg.header.stamp = rosmaster_time
            msg.nodeID = i+1
            msg.radians = 0
            # msg.gear_reduction = 0

            if i<3: # for the right side joints
                msg.velocity = - omega_right        # this value is negated as the rotational side is inverse
            else:   # for the left side joints
                msg.velocity = omega_left

            # append the message in the command list
            command_message_array[i] = msg

        # save the mounted array to be published in ROS
        command_message_list.movement_command = command_message_array

        # returns the command
        return command_message_list

    # --- generates joints commands values for the legged (tripod) locomotion mode ---
    # THIS IS NOT DONE YET!
    def control_update_sixLegs(self, rosmaster_time):

        # the returning list
        command_message_list = MovementArray()

        command_message_array = [Movement()]*6

        # isolating relevant variables for the sake of readability
        phis_min = self.locomotionParamDictionary['legged_phis_min']
        phis_max = self.locomotionParamDictionary['legged_phis_max']
        speed_translational_max = self.locomotionParamDictionary['espeleo_maxTranslationalSpeed']

        cmd_linear = self.locomotionControlDictionary['linearSetPoint']
        cmd_angular = self.locomotionControlDictionary['angularSetPoint']

        # this code uses the Bueheler's Clock concept to compute the joints speeds
        # CORRECT THAT CORRECT THAT!
        # computing each side slow angle

        # pre treatment
        phis_mean = (phis_max + phis_min) / 2
        phis_half = (phis_max - phis_min) / 2

        # computing each side slow phase angle
        phis_right = phis_mean + cmd_angular * phis_half
        phis_left = phis_mean - cmd_angular * phis_half

        # computing the T value
        T = cmd_linear * speed_translational_max 

        # computes the derivative of Buehler's Clock curve (joints speed)
        #right side
        if T != 0:
            omega_right_slow = (2 * phis_right) / (T/2)
            omega_right_fast = (2 * np.pi - omega_right_slow) / (T/2)
            #left side
            omega_left_slow = (2 * phis_left) / (T/2)
            omega_left_fast = (2 * np.pi - omega_left_slow) / (T/2)
        else:
            omega_right_slow = 0
            omega_right_fast = 0
            omega_left_slow = 0
            omega_left_fast = 0

        # computes each joint velocity setpoint
        # the signal is inverter for the left side as these joints are inverse
        omega_joint = [None]*6
        
        # assembles the returning velocity list for all motors
        for i in range(0,6):

            msg = Movement()
            
            msg.header.stamp = rosmaster_time
            msg.nodeID = i+1
            msg.radians = 0
            # msg.gear_reduction = 0
            msg.velocity = omega_joint[i]

            # append the message in the command list
            command_message_array[i] = msg

        # save the mounted array to be published in ROS
        command_message_list.movement_command = command_message_array

        # returns the command
        return command_message_list


    # ==== LOGISTIC METHODS =====

    # --- updates the locomotion control parameters
    def set_locomotionControlParameters(self, espeleo_input):

        self.locomotionControlDictionary['linearSetPoint'] = espeleo_input['linear']
        self.locomotionControlDictionary['angularSetPoint'] = espeleo_input['angular']
        self.locomotionControlDictionary['lastValidTime'] = espeleo_input['time_stamp']

    # --- Method for setting encoders values ---
    def set_joints_position(self, array_encoder):
        
        # saves the value inside a global variable
        self.joints_position = array_encoder


    # --- Method for setting the locomotion Param dictionary
    def update_locomotionParamDictionary(self, locomotionParamDictionary):

        # updates the locomotion dictionary
        self.locomotionParamDictionary = locomotionParamDictionary

        # updates wheeled mode kinematic A matrix
        self.wheeled_kinematic_A = self.compute_kinematicAMatrix(self.locomotionParamDictionary['wheeled_kinematic_lambda'], self.locomotionParamDictionary['wheeled_radius'], self.locomotionParamDictionary['wheeled_kinematic_ycir'])


    # ======================== MATH SUPPORT METHODS ===============================

    # -- Method for compute wheeled mode A matrix
    @staticmethod
    def compute_kinematicAMatrix(var_lambda, wheel_radius, ycir):

        # kinematic A matrix (again, see Filipe Rocha dissertation for + info)
        matrix_A = np.array([[var_lambda*wheel_radius/2, var_lambda*wheel_radius/2], \
                                            [(var_lambda*wheel_radius)/(2*ycir), -(var_lambda*wheel_radius)/(2*ycir)]])
 
        # returns matrix A
        return matrix_A

    # --- Method for computing the derivative of Buehler's Clock curver
    @staticmethod
    def compute_BuehlerDerivatives(phi_s, T, signal):
        

        pass

   
# ==== For class testing purposes
if __name__ == '__main__':

    # create the object
    loc_obj = bib_espeleo_locomotion()

    # create a fake input command
    loc_obj.set_locomotionControlParameters({'time_stamp':0, 'command_type':1, 'linear':1, 'angular':1})

    # request to compute the joints command
    ret_list = loc_obj.control_update_sixWheels()

    print(ret_list)