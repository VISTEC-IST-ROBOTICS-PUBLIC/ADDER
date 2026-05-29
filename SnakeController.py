"""
SnakeController inherits the DynamixelController class. SnakeController generates the gaits for ADDER, which
would then call the DynamixelController in order to control the motors. To get sensory feedback for data logging,
the I2CSensorCollector class is used to obtain the voltage and current data from the INA219 sensor, along with 
the accelaration and the gyroscope data from the MPU6050 sensor. 
"""

#DynamixelController to control the motors
from DynamixelController import DynamixelController

#Imports a class that collects data from two different sensors (INA219 for current and MPU6050 for acceleration) recorded through I2C communication
from Sensors import I2CSensorCollector

#Standard libraries
import time
import math


class SnakeController(DynamixelController):
    """
    This class generates gait commands to control ADDER. It inherits the DynamixelController class
    in order to control the snake robot's dynamixel motor.
    """
    def __init__(self, dxl_id, operating_mode = 4, 
                 profile_velocity = 0, profile_acceleration = 0, max_translation = 20, translation_bias = 10,
                 individual_translation_bias: list = [0,0,0,0,0,0,0,0,0,0]):      
        """
        Initialize the SnakeController.

        Parameters
        ----------
        dxl_ids : list[int], optional
            List of Dynamixel IDs to control. Defaults to IDs 0-19.

        operating_mode : int, optional
            Operating mode to apply to all motors. See
            `set_operating_mode()` from DynamixelController for supported modes.
            Default is 4.

        profile_velocity : int, optional
            Profile velocity applied to all motors. Refer to the
            Dynamixel e-Manual for valid values. Default is 0.

        profile_acceleration : int, optional
            Profile acceleration applied to all motors. Refer to the
            Dynamixel e-Manual for valid values. Default is 0.

        max_translation : float, optional
            Sets the maximum translation command for the double-spiral joint.
            This value is adjusted depending on the double-spiral property

        translation_bias : float, optional
            Sets the translation_bias for ALL double-spiral joints.

        individual_translation_bias : list[float], optional
            Sets the translation_bias for each double-spiral joints.
        
        
        Output
        ----------
        None
        """
        #Width of the double-spiral.
        self.__w = 33
        #Radius of the double-spiral.
        self.__r = 7.5

        #Rest position of the dynamixel motor.
        self.added_zero = 2047

        #Initialize translation values.
        self.max_translation = max_translation
        self.translation_bias = translation_bias
        self.individual_translation_bias = individual_translation_bias

        #Checks if number of joints has the same number of translation biases.
        if len(dxl_id)//2 != len(individual_translation_bias):
            raise ValueError(f"Joints: {len(dxl_id)//2}\tTranslation Biases: {len(individual_translation_bias)}")

        #Sets offset for the DynamixelController.
        self.offset = [0 for _ in range(len(dxl_id))]

        #Initialize I2CSensorCollector and data logging containers.
        self.sensor = I2CSensorCollector()
        self.ina  = []
        self.mpu  = []
        self.time = []

        #Set start time for the data logging.
        self.start_time = time.time()

        #Initialize parent class.
        super().__init__(dxl_id, operating_mode, self.offset, profile_velocity, profile_acceleration)

        #Start initialization of the joints.
        self.init()
       
    @property
    def w(self):
        """
        Return the width of the double-spiral.

        Parameters
        ----------
        None
       
        Output
        ----------
        Double-spiral width : float
        """
        return self.__w
    
    @property
    def r(self):
        """
        Return the radius of the double-spiral.

        Parameters
        ----------
        None
       
        Output
        ----------
        Double-spiral radius : float
        """
        return self.__r       

    def init(self):
        """
        Initializes the joints. The resting position of the double-spiral joint is when it is compressed.
        This allows a forward translation motion which were used in the literature. 

        Parameters
        ----------
        None
       
        Output
        ----------
        Double-spiral width : float
        """
        print("\n\n\nStart Pos")

        #Send a command that compresses all of the joints.
        self.joint_command([[0, self.translation_bias] for _ in range(round(len(self.DXL_ID)/2))])

        #Delay before sending the next command.
        time.sleep(2)

    def joint_command(self, command):
        """
        This allows a forward translation motion which were used in the literature. 

        Parameters
        ----------
        command : list[list], required
            Joint command for the double-spiral joint.
            command = [[angle_1, pos_1], [angle_2, pos_2], ... , [angle_n, pos_n]]
            angle is in degrees
            pos is in millimeters (range is 5-20 mm/2)
       
        Output
        ----------
        None
        """
        #Checks if the command has the same length as half of the number of motors (since 1 double-spiral joint is controlled by 2 motors).
        if len(command) != len(self.DXL_ID)/2:
            raise ValueError(f"ERROR!\nlen(command): {len(command)}\tlen(self.DXL_ID)/2: {len(self.DXL_ID)/2}")
        
        print(command)

        #Converts joint command into a motor command.
        listOfMotorCommand = self.__joint_to_motor_command(command)

        #Actuates the motor according to the motor command.
        self.__move_joint(listOfMotorCommand)

    def __joint_to_motor_command(self, command):
        """
        Converts joint commands to motor command.

        Parameters
        ----------
        command : list[list], required
            Joint command for the double-spiral joint.
            command = [[angle_1, pos_1], [angle_2, pos_2], ... , [angle_n, pos_n]]
            angle is in degrees
            pos is in millimeters (range is 5-20 mm/2)
       
        Output
        ----------
        Motor command : list
            List of motor command
        """
        #Container for the motor command.
        listOfMotorCommand = []

        #Iterates through each joints and translational biases.
        for joint, i_tb in zip(command, self.individual_translation_bias):
            #Convert joint angle to radians.
            angle = math.radians(joint[0])     

            #Convert joint position to a translational command.
            position = -joint[1] + self.max_translation + i_tb

            #Convert to motor command for left and right into degrees, and add offset.
            motor_l = math.degrees(((self.w*angle + position)/self.r))      
            motor_r = math.degrees(((self.w*angle - position)/self.r)) 

            #Command ratio such that maximum_dynamixel_command/complete_circle_angle.
            commandRatio = 4095/360            
    
            #multiply ratio and round since Dynamixel motor commands are integers.
            motor_l = -round(motor_l * commandRatio)  + self.added_zero    
            motor_r = round(motor_r * commandRatio)  + self.added_zero

            #Add motor commands to the container.
            listOfMotorCommand.append(motor_l)
            listOfMotorCommand.append(motor_r)

        #Return the container
        return listOfMotorCommand

    def __move_joint(self, command):
        """
        Rotates the motors according to the command.
        Logs sensor data for data collection.

        Parameters
        ----------
        command : list, required
            Motor command for the Dynamixel motors.
            
       
        Output
        ----------
        None
        """
        #Calls the parent class's modified_rotate() to actuate the Dynamixel motors.
        self.modified_rotate(command)

        #Log sensor data.
        ina_dict = self.sensor.read_ina219()
        mpu_dict = self.sensor.read_mpu6050()

        #Alert user if battery is running out. Shutdown if too low.
        if ina_dict['voltage'] < 6.0:
            print(f"WARNING! INSUFFICIENT BATTERY AT {ina_dict['voltage']}V")
        elif ina_dict['voltage'] < 5.5:
            print(f'SHUTDOWN!')
            self.rest()
        else:
            print(f"Battery: {round(ina_dict['voltage'], 2)}V")
            print(f"Current: {round(ina_dict['current'], 2)}A")
            print()
        
        #Add sensor data to the container.
        self.ina.append(ina_dict)
        self.mpu.append(mpu_dict)       
        self.time.append(time.time() - self.start_time)

        #Add 1ms of delay.
        time.sleep(0.001)    



    ##################
    ###Snake Gaits####
    ##################    
    def serpentine(self, amplitude = 30, angular_frequency = 1, phase_shift = 30, orientation = 0, stopping_constant = 10, bias = 0):
        """
        Serpentine gait derived from a serpenoid equation.

        Parameters
        ----------
        amplitude : float, optional
            Maximum joint angle of the double-spiral.

        angular_frequency : float, optional
            Angular frequency of the sinusoidal wave.

        phase_shift : float, optional
            Phase shift between joints.

        orientation : int, optional
            Determines whether serpenoid wave is propagated on the yaw or pitch joint.
                Yaw: 0
                Ptich: 1

        stopping_constant : int, optional
            Duration of the gait.

        bias : int, optional
            Bias of the serpnoid wave.
            Could be used for steering in the future work.


        Output
        ----------
        None
        """
        #Record starting time
        start_time = time.time()        

        #iterate
        for _ in range(stopping_constant): 
            #Command container is reset every iteration
            command = []               

            #Get t     
            t = time.time() - start_time     

            #iterate through all joints on preferred orientation
            for joint in range(round(len(self.DXL_ID)/4)): 
                #Get joint angle
                angle = amplitude * math.sin(angular_frequency * t + joint * phase_shift) + bias     
                
                #Append command with skips (inactive joints) according to the orientation
                command = self.serpentine_skipping_joint(round(angle, 2), orientation)  + command     
            
            #Reverse the command container so the signal propagates from the head to the tail.
            command.reverse()

            #Send command to the motors
            self.joint_command(command) 


    def serpentine_skipping_joint(self, command, orientation = 0):
        """
        Completes the joint command by adding translation command on one joint 
        and adding rest on the other depending on the orientation

        Parameters
        ----------
        command : float, required
            Joint command for the active joint. 
            The rest will be set at their resting position.

        orientation : int, optional
            Determines whether serpenoid wave is propagated on the yaw or pitch joint.
                Yaw: 0
                Ptich: 1


        Output
        ----------
        Joint command : list[list]
            Full joint command with rest for inactive joints
        """
        #Rest joint on one [0, self.translation_bias] and active joint on the other [command,20] depending 
        #on the orientation.
        if orientation == 0:
            return [[command,20], [0,self.translation_bias]]
        else:
            return [[0,self.translation_bias], [command, 20]]
            
    def sidewinding(self, amplitude_tangent = 30, angular_frequency_tangent = 1, phase_shift_tangent = 30, 
                    amplitude_normal = 15, angular_frequency_normal = 1, phase_shift_normal = 30, stopping_constant = 10):
        """
        Sidewinding gait generated from two serpenoid equation for the yaw and pitch axes.

        Parameters
        ----------
        amplitude_tangent : float, optional
            Maximum joint angle for the
            yaw axes of ADDER.

        angular_frequency_tangent : float, optional
            Angular frequency of the yaw axes.

        phase_shift_tangent : float, optional
            Phase shift between joints of the yaw axes.

        amplitude_normal : float, optional
            Maximum joint angle for the
            pitch axes of ADDER.

        angular_frequency_normal : float, optional
            Angular frequency of the pitch axes.

        phase_shift_normal : float, optional
            Phase shift between joints of the pitch axes.

        stopping_constant : int, optional
            Duration of the gait.


        Output
        ----------
        None
        """
        #Record starting time
        start_time = time.time()        

        #Iterate until the stopping_constant.
        for _ in range(stopping_constant):  
            #Command container is reset every iteration
            command = []       

            #record current time
            t = time.time() - start_time   

            #Iterate through all joints.
            for joint in range(round(len(self.DXL_ID)/4)):      
                #Get angle for the yaw
                angle_tangent = amplitude_tangent * math.sin(angular_frequency_tangent * t + joint * phase_shift_tangent)     
                
                #Get angle for the yaw
                angle_normal = amplitude_normal * math.sin(angular_frequency_normal * t + joint * phase_shift_normal)
               
                #Append command with preset translation position.
                command = command + [[angle_tangent,20], [angle_normal, 20]] 
            
            #Send command to the motors 
            self.joint_command(command) 

    def sinusoidal_rectilinear(self, amplitude = 20, angular_frequency = 1, phase_shift = [math.pi/6 for _ in range(10)], stopping_constant = 100):
        """
        Rectilinear gait derived from a sinusoidal wave.

        Parameters
        ----------
        amplitude : float, optional
            Maximum translation of the double-spiral.

        angular_frequency : float, optional
            Angular frequency of the sinusoidal wave.

        phase_shift : list[float], optional
            Phase shift between joints.

        stopping_constant : int, optional
            Duration of the gait.


        Output
        ----------
        None
        """
        #Start time for the sinusoidal wave.
        start_time = time.time()

        #Iterate until flag_counter reaches the limit.
        for _ in range(stopping_constant):  
            #Command container which resets every iteration.
            command = []                  

            #Get current time for the sinusoidal wave.
            t = time.time()    - start_time      

            #Iterate through all joints.
            for joint in range(round(len(self.DXL_ID)/2)):      
                #Get translation through sinusoidal wave. Clip values below the amplitude to have a 
                #rest time for the translation joint similar to the proposed gait in the literature.
                translation = max(amplitude * math.sin(angular_frequency * t + joint * phase_shift[joint])  + amplitude, amplitude)   
                
                #Append command to he container
                command.append([0, translation])
            
            #Send command to the motors
            self.joint_command(command)
    
    def concertina(self, extend = 20, compress = 10, max_angle = 60, number_of_transitions = 16, number_of_commands = 10):
        """
        Concertina gait derived from a sequence of commands. All yaw joints rotates while the pitch joints translates.

        Parameters
        ----------
        extend : float, optional
            Maximum joint translation of the double-spiral.
        
        compress : float, optional
            Rest joint translation of the double-spiral.

        max_angle : float, optional
            Maximum joint angle of the double-spiral.

        number_of_transitions : int, optional
            Determines the number of commands between the four main commands: rest, front_compress, full_compress, and back_compress.

        number_of_commands : int, optional
            Duration of the gait.


        Output
        ----------
        None
        """
        #Main concertina commands
        #No movement from all of the joints
        rest = [extend, 0] * 5
        #The joints on the middle segment until the head of ADDER will rotate and compress for the yaw and pitch joints, respectively. The other half will be at rest position.
        front_compress = [extend,              0,   extend,          0,   extend, -max_angle //2, compress, max_angle, compress, -max_angle//2]
        #All the joints of ADDER will rotate and compress for the yaw and pitch joints, respectively.
        full_compress  = [compress, -max_angle//2, compress, max_angle, compress,     -max_angle, compress, max_angle, compress, -max_angle//2]
        #The joints on the tail until the middle segment  of ADDER will rotate and compress for the yaw and pitch joints, respectively. The other half will be at rest position.
        back_compress  = [compress, -max_angle//2, compress, max_angle, compress, - max_angle//2,   extend,          0,   extend,  0]

        #Generate transition commands between the main commands.
        t1 = self.smooth_transition(rest, front_compress, number_of_transitions)
        t2 = self.smooth_transition(front_compress, full_compress, number_of_transitions)
        t3 = self.smooth_transition(full_compress,back_compress, number_of_transitions)
        t4 = self.smooth_transition(back_compress, rest, number_of_transitions)
        
        #Generate the whole sequence of command
        full_command = [rest]
        for transition in t1:
            full_command.append(transition)

        full_command.append(front_compress)
        for transition in t2:
            full_command.append(transition)

        full_command.append(full_compress)
        for transition in t3:
            full_command.append(transition)

        full_command.append(back_compress)
        for transition in t4:
            full_command.append(transition)

        #Add rest on both sides of the sequence
        list_of_command = [rest] *2 + full_command + [rest] *2
        final_command = []

        #Fix the current command to the appropritate format of joint_command()
        for command in list_of_command:
            added_command = []
            for i in range(len(command)//2):
                added_command.append([0, round(command[2*i],2)])
                added_command.append([round(command[2*i + 1]), 20])

            final_command.append(added_command)

        #Iterate this sequence for the number of commands.
        for _ in range(number_of_commands):
            for command in final_command:
                print("\nCurrent:")
                print(self.joint_command(command))
    

    def smooth_transition(self,list_1, list_2, number_of_list_transitions): 
        """
        Generates a smooth transition command for the concertina gait.

        Parameters
        ----------
        list_1 : list, requried
            Starting command.
        
        list_2 : list, requried
            End command.

        number_of_list_transitions : int, requried
            Determines the number of commands between the four main commands: rest, front_compress, full_compress, and back_compress.


        Output
        ----------
        transition commands : list[list]
            List of transition commands between list_1 and list_2.
            The number of commands dependes on the number_of_list_transitions.
        """
        #Check if both lists are of equal length
        if len(list_1) != len(list_2):
            raise ValueError("Both input lists must have the same length")
        
        #Transition container
        transitions = []

        #Generate the transition commands and append them to the container
        for step in range(1, number_of_list_transitions + 1):
            ratio = step / (number_of_list_transitions + 1)
            intermediate = [
                round((1 - ratio) * a + ratio * b,2) for a, b in zip(list_1, list_2)
            ]
            transitions.append(intermediate)
        
        return transitions

    
    ###
    ###UTILITIES
    ###

    def rest(self):
        """
        Closes Dynamixel communication and returns sensor data

        Parameters
        ----------
        None

        Output
        ----------
        sensor data : list, list, list
            returns time, ina219, and mpu6050 data for post-processing
        """
        print("\n\nClosing Controller")

        #Rests all of the motors.
        command = [[0,self.max_translation] for _ in range(round(len(self.offset)/2))]
        self.joint_command(command)

        #Delay.
        time.sleep(2)

        #Close the communication.
        self.close()

        #Return time and sensor data.
        return self.time, self.ina, self.mpu


    
