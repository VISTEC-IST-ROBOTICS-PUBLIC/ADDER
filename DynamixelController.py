"""
This code is uses dynamixel_sdk to control Dynamixel motors. Although ADDER uses Dynamixel XL330-M288-T, the DynamixelController class can still be used
for other Dynamixel motors by modifying the control table address. This class is not tested for controlling different Dynamixel motors. To control multiple
motors, it uses GroupSyncRead and GroupSyncWrite to read sensor data from the motors and send commands to the actuators, respectively. Since ADDER uses
20 Dynamixel motors, modified_rotate() must be used since this method send command two motors at a time adding a delay before sending the command to the
next set of motors. However, receiving all sensory signals is not recommended due to the large number of motors.
"""

#####################################################################
#Cross-platform######################################################
#Keyboard input######################################################
#####################################################################
import os, sys, ctypes

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
#####################################################################
#####################################################################

#dynamixel_sdk for controlling dynamixel motors
from dynamixel_sdk import *

#Used to add delays
import time

class DynamixelController:
    """
    This class is designed to communicate with multiple dynamixel motors to control them and receive their sensory outputs.
    """
    # Control table address. Current parameters are set to control Dynamixel XL330-M288-T
    ADDR_OPERATING_MODE         = 11
    ADDR_HOMING_OFFSET          = 20
    ADDR_VELOCITY_LIMIT         = 44
    ADDR_PRO_SHUTDOWN           = 63
    ADDR_PRO_TORQUE_ENABLE      = 64               
    ADDR_PRO_LED_RED            = 65
    ADDR_PRO_HARDWARE_ERROR_STATUS = 70
    ADDR_PRO_VEL_I_GAIN         = 76
    ADDR_PRO_VEL_P_GAIN         = 78
    ADDR_PRO_POS_D_GAIN         = 80
    ADDR_PRO_POS_I_GAIN         = 82
    ADDR_PRO_POS_P_GAIN         = 84
    ADDR_PRO_FEED_2ND_GAIN      = 88
    ADDR_PRO_FEED_1ST_GAIN      = 90
    ADDR_PRO_GOAL_PWM           = 100
    ADDR_PRO_GOAL_CURRENT       = 102
    ADDR_PRO_GOAL_VELOCITY      = 104
    ADDR_PRO_GOAL_POSITION      = 116
    ADDR_PRO_PRESENT_PWM        = 124
    ADDR_PRO_PRESENT_CURRENT    = 126
    ADDR_PRO_PRESENT_VELOCITY   = 128
    ADDR_PRO_PRESENT_POSITION   = 132
    ADDR_PRO_PROFILE_ACCELARATION = 108
    ADDR_PRO_PROFILE_VELOCITY   = 112
    

    # Data Byte Length. Current parameters are set to control Dynamixel XL330-M288-T
    LEN_PRO_LED_RED             = 1
    LEN_PRO_VEL_I_GAIN          = 2
    LEN_PRO_VEL_P_GAIN          = 2
    LEN_PRO_POS_D_GAIN          = 2
    LEN_POS_I_GAIN              = 2
    LEN_PRO_POS_P_GAIN          = 2
    LEN_PRO_FEED_2ND_GAIN       = 2
    LEN_PRO_FEED_1ST_GAIN       = 2
    LEN_PRO_GOAL_PWM            = 2
    LEN_PRO_GOAL_CURRENT        = 2
    LEN_PRO_PRESENT_CURRENT     = 2
    LEN_PRO_GOAL_POSITION       = 4
    LEN_PRO_PRESENT_POSITION    = 4
    LEN_PRO_GOAL_VELOCITY       = 4
    LEN_PRO_PRESENT_VELOCITY    = 4
    LEN_PRO_HOMING_OFFSET       = 4
    LEN_PRO_VELOCITY_LIMIT      = 4
    LEN_PRO_PROFILE_ACCELARATION= 4
    LEN_PRO_PROFILE_VELOCITY    = 4

    # Protocol version
    PROTOCOL_VERSION            = 2.0

    # Default setting
    BAUDRATE                    = 57_600             # Dynamixel default baudrate : 57600
    DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 50               # Dynamixel moving status threshold
    
    
    def __init__ (self, dxl_ids = None, operating_mode = 3, offset = None, profile_velocity = 0, profile_acceleration = 0, limit_vel = 445):
        """
        Initialize the DynamixelController.

        Parameters
        ----------
        dxl_ids : list[int], optional
            List of Dynamixel IDs to control. Defaults to IDs 0-19.

        operating_mode : int, optional
            Operating mode to apply to all motors. See
            `set_operating_mode()` for supported modes.
            Default is 3.

        offset : list[float], optional
            Position offsets added to commanded goal positions.
            Must contain one value per motor. Defaults to all zeros.

        profile_velocity : int, optional
            Profile velocity applied to all motors. Refer to the
            Dynamixel e-Manual for valid values. Default is 0.

        profile_acceleration : int, optional
            Profile acceleration applied to all motors. Refer to the
            Dynamixel e-Manual for valid values. Default is 0.

        limit_vel : int, optional
            Sets velocity limit to all motors. Default is 445

        
        Output
        ----------
        None
        """
        #Default values for dxl_ids and offset if None
        if dxl_ids is None:
            dxl_ids = list(range(20))
        if offset is None:
            offset = [0] * 20

        #Set all init values
        self.DXL_ID = dxl_ids   #Make sure to set all Dynamixel IDs in Dynamixel Wizard beforehand
        self.operating_mode = operating_mode
        self.offset = offset

        #Initialize PortHandler and PacketHandler for handling serial communication and packet encoding/decoding, respectively.
        self.port_handler = PortHandler(self.DEVICENAME)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)
        
        #Sets velocity limit of all dynamixel motors
        self.set_velocity_limit(limit_vel=limit_vel)

        #Initalizes GroupSyncRead for reading the positions, velocities, and current of all motors.
        self.position_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
        self.velocity_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, self.ADDR_PRO_PRESENT_VELOCITY, self.LEN_PRO_PRESENT_VELOCITY)
        self.current_sync_read  = GroupSyncRead(self.port_handler, self.packet_handler, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT)

        #Initalizes GroupSyncWrite for writing the positions, velocities, and current of all motors.
        self.position_sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, self.ADDR_PRO_GOAL_POSITION, self.LEN_PRO_GOAL_POSITION)
        self.velocity_sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, self.ADDR_PRO_GOAL_VELOCITY, self.LEN_PRO_GOAL_VELOCITY)
        self.current_sync_write  = GroupSyncWrite(self.port_handler, self.packet_handler, self.ADDR_PRO_GOAL_CURRENT,  self.LEN_PRO_GOAL_CURRENT)
        
        #Sets profile velocities and accelerations of all motors.
        self.profile_velocity = profile_velocity
        self.profile_acceleration = profile_acceleration

        #Initializes the operating mode, profile velocity, and profile acceleration of dynamixel motors.
        self.initialize(operating_mode, profile_velocity, profile_acceleration)

    def initialize(self, operating_mode, profile_velocity, profile_acceleration):
        """
        Initialization routine for all motors.

        Parameters
        ----------
        operating_mode : int, required
            Sets the operating mode of the motors. 
            See `set_operating_mode()` for supported modes.
        
        profile_velocity : int, required
            Profile velocity applied to all motors. Refer to the
            Dynamixel e-Manual for valid values. Default is 0.

        profile_acceleration : int, required
            Profile acceleration applied to all motors. Refer to the
            Dynamixel e-Manual for valid values. Default is 0.


        Output
        ----------
        None
        """

        print("Start Initialization...")

        #Opens port
        if self.port_handler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        #Sets baudrate
        if self.port_handler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate\n")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
        
        #Disables torque to change the operating mode.
        self.disable_torque()

        #Sets operating mode.
        self.set_operating_mode(operating_mode)

        #Adds parameters on all motors for sensory inputs: position, velocity, current.
        self.add_parameters()
        
        #Enables torque to control the Dynamixel motors.
        self.enable_torque()
        
        #Sets the profile velocity on all motors.
        print("\nSetting Profile Velocity")
        self.set_profile_velocity(profile_velocity)

        #Sets the profile acceleration on all motors.
        print("\nSetting Profile Acceleration")
        self.set_profile_acceleration(profile_acceleration)

    def add_parameters(self):
        """
        Adds parameters on each dynamixel motors for reading their sensory outputs

        Parameters
        ----------
        None


        Output
        ----------
        None
        """
        #Iterates through each motors.
        for id in self.DXL_ID:
            #Adds param for position.
            dxl_add_param_result = self.position_sync_read.addParam(id)
            #Verifies if the action failed.
            if dxl_add_param_result != True:
                print(f"ID: {id} addparam on position failed")
            
            #Adds param for velocity.
            dxl_add_param_result = self.velocity_sync_read.addParam(id)
            #Verifies if the action failed.
            if dxl_add_param_result != True:
                print(f"ID: {id} addparam on position failed")

            #Adds param for current.
            dxl_add_param_result = self.current_sync_read.addParam(id)
            #Verifies if the action failed.
            if dxl_add_param_result != True:
                print(f"ID: {id} addparam on current failed")

    def enable_torque(self):
        """
        Enables torque on all motors locking them and enables them to move through commands.

        Parameters
        ----------
        None

        
        Output
        ----------
        None
        """
        #List that will collect all motors that cannot be enabled.
        idError = []

        #Iterates through each motors.
        for id in self.DXL_ID:
            #Sends a command that will enable the torque of the motor. Will received outputs whether the communication is successful and what is the error if it failed.
            dxlCommunicationResult, dxlHardwareError = self.packet_handler.write1ByteTxRx(self.port_handler, id, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)

            #If communication is not successful, show the error and append the motor ID to the list.
            if dxlCommunicationResult != COMM_SUCCESS:
                print(f"{self.packet_handler.getTxRxResult(dxlCommunicationResult)}")
                idError.append(id)
            
            #If there is hardware error, show this and append the motor ID to the list.
            elif dxlHardwareError != COMM_SUCCESS:
                print(f"{self.packet_handler.getRxPacketError(dxlHardwareError)}")
                idError.append(id)

        #Shows if enabling the torque is successful or not.
        if len(idError) == 0:
            print("Dynamixel motors has successfully connected")      
        else:
            print("Error occured on ID: ", end='')
            for id in idError:
                print(f"{id}", end = ', ')

    def disable_torque(self):
        """
        Disables torque on all motors.

        Parameters
        ----------
        None


        Output
        ----------
        None
        """
        #List that will collect all motors that cannot be enabled.
        idError = []

        #Iterates through each motors.
        for id in self.DXL_ID:
            #Sends a command that will enable the torque of the motor. Will received outputs whether the communication is successful and what is the error if it failed.
            dxlCommunicationResult, dxlHardwareError = self.packet_handler.write1ByteTxRx(self.port_handler, id, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)

            #If communication is not successful, show the error and append the motor ID to the list.
            if dxlCommunicationResult != COMM_SUCCESS:
                print(f"{self.packet_handler.getTxRxResult(dxlCommunicationResult)}")
                idError.append(id)
            
            #If there is hardware error, show this and append the motor ID to the list.
            elif dxlHardwareError != COMM_SUCCESS:
                print(f"{self.packet_handler.getRxPacketError(dxlHardwareError)}")
                idError.append(id)

        #Shows if disabling the torque is successful or not.
        if len(idError) == 0:
            print("Dynamixel motors has successfully disconnected")      
        else:
            print("Error occured on ID: ", end='')
            for id in idError:
                print(f"{id}", end = ', ')


    def set_operating_mode(self, mode):
        """
        Set Dynamixel operating mode. Values are based on the control 
        table of Dynamixel XL330-M288-T. Modify. List of valid values
        if another model of Dynamixel motor is used.

        Parameters
        ----------

        mode : int, required 
            operating mode of the Dynamixel motors.
            Possible values:
                Current Control: 0
                Velocity Control: 1
                Position Control: 3
                Extended Position Control: 4
                Current-based Position Control: 5
                PWM Control: 16
        
        
        Output
        ----------
        None
        """
        print("Setting Operating Mode...")

        #Raises a value error if the mode is not part of valid values.
        valid_values = [0, 1, 3, 4, 5, 16]
        if mode not in valid_values:
            raise ValueError(f"Invalid operating mode {mode}. Valid modes: {valid_values}")

        # Torque must be disabled before changing mode.
        self.disable_torque()

        #Iterates through all the motors
        for id in self.DXL_ID:
            #Sends a command that sets the operating mode of the motor. Will received outputs whether the communication is successful and what is the error if it failed.
            dxlCommunicationResult, dxlHardwareError = self.packet_handler.write1ByteTxRx(self.port_handler, id, self.ADDR_OPERATING_MODE, mode)

            #If communication is not successful, show the error.
            if dxlCommunicationResult != COMM_SUCCESS:
                print(f"[ID {id}] COMM ERROR: {self.packet_handler.getTxRxResult(dxlCommunicationResult)}")
                
            #If there is hardware error, show the error.
            elif dxlHardwareError != COMM_SUCCESS:
                print(f"[ID {id}] PACKET ERROR: {self.packet_handler.getRxPacketError(dxlHardwareError)}")
                
            #Inform that the operating mode has been set.
            print(f"[ID {id}] Operating mode set to {mode}")

            #Read hardware error status.
            hw_error, comm_result2, dxl_error2 = self.packet_handler.read1ByteTxRx(self.port_handler, id, self.ADDR_PRO_HARDWARE_ERROR_STATUS)

            #Show the errors.
            if comm_result2 != COMM_SUCCESS:
                print(f"[ID {id}] HW ERROR READ FAILED: {self.packet_handler.getTxRxResult(comm_result2)}")
                
            if dxl_error2 != COMM_SUCCESS:
                print(f"[ID {id}] HW PACKET ERROR: {self.packet_handler.getRxPacketError(dxl_error2)}")
                
            if hw_error != COMM_SUCCESS:
                print(f"[ID {id}] HARDWARE ERROR STATUS: {hw_error}")

                if hw_error & 1:
                    print("  - Input Voltage Error")
                if hw_error & 2:
                    print("  - Overheating Error")
                if hw_error & 4:
                    print("  - Motor Encoder Error")
                if hw_error & 8:
                    print("  - Electrical Shock Error")
                if hw_error & 16:
                    print("  - Overload Error")
            print("========")

        #Re-enable torque.
        self.enable_torque()

        #Set the profile velocity
        print("\nSetting Profile Velocity")
        self.set_profile_velocity(self.profile_velocity)

        #Set the profile acceleration
        print("\nSetting Profile Acceleration")
        self.set_profile_acceleration(self.profile_acceleration)

    def get_all_data(self):
        """
        Returns the present position, velocity, and current of all motors.

        Parameters
        ----------
        None


        Output
        ----------
        list[list]: [list_of_positions, list_of_velocities, list_of_currents]
        """
        #Check if there is communication for position.
        dxl_comm_result = self.position_sync_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))

        #Check if there is communication for velocity.
        dxl_comm_result = self.velocity_sync_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))
        
        #Check if there is communication for current.
        dxl_comm_result = self.current_sync_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))

        # Storage of all sensor data.
        list_of_positions = []
        list_of_velocities = []
        list_of_currents = []

        #Iterate through each motors.
        for dxl_id in self.DXL_ID:
            #Check whether it is possible to read position data.
            if not self.position_sync_read.isAvailable(dxl_id, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION):
                print(f"ID: {dxl_id} isAvailable on position failed")

            #Check whether it is possible to read velocity data.
            if not self.velocity_sync_read.isAvailable(dxl_id, self.ADDR_PRO_PRESENT_VELOCITY, self.LEN_PRO_PRESENT_VELOCITY):
                print(f"ID: {dxl_id} isAvailable on velocity failed")

            #Check whether it is possible to read current data.
            if not self.current_sync_read.isAvailable(dxl_id, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT):
                print(f"ID: {dxl_id} isAvailable on current failed")            

            #Read position, velocity, and current.
            position = self.position_sync_read.getData(dxl_id, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
            velocity = self.velocity_sync_read.getData(dxl_id, self.ADDR_PRO_PRESENT_VELOCITY, self.LEN_PRO_PRESENT_VELOCITY)
            current = self.current_sync_read.getData(dxl_id, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT)

            #Append sensory data to their respective lists
            list_of_positions.append(position)
            list_of_velocities.append(velocity)
            list_of_currents.append(current)

        #return sensor data in one list
        return([list_of_positions, list_of_velocities, list_of_currents])

    def get_all_current(self):
        """
        Returns present current data from all the motors.

        Parameters
        ----------
        None


        Output
        ----------
        list: list_of_currents
        """
        #Check if there is communication.
        dxl_comm_result = self.current_sync_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))

        #Storage of current data.                                                 
        list_of_current = []

        #Iterate through all dynamixels and check if they're available.
        for id in self.DXL_ID:
            #Check if the current data is available.
            if self.current_sync_read.isAvailable(id, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT) != COMM_SUCCESS:
                print(f"ID: {id} isAvailable on current failed")

            #Get current.
            current = self.current_sync_read.getData(id, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT)

            #Append current the the list of currents.
            list_of_current.append(self.to_signed_16bit(current))
        
        #Return current data of all motors.
        return list_of_current
    
    @staticmethod
    def to_signed_16bit(value):
        """
        Convert an unsigned 16-bit value to its signed 16-bit representation.

        Parameters
        ----------
        None


        Output
        ----------
        float: converted signed value from 16bit
        """
        return value - 0x10000 if value > 0x7FFF else value

    def set_velocity_limit(self, limit_vel):
        """
        Set velocity limit of all motors.

        Parameters
        ----------
        limit_vel : int, required
            limit velocity of the motors
            0 > limit_vel > 2047


        Output
        ----------
        None
        """
        #Check if limit_vel is within range.
        if 0 > limit_vel > 2047:
            raise ValueError(f"Velocity out of bounds. limit_vel = {limit_vel}")
        
        #Set limit velocity.
        self.limit_vel = limit_vel
        
        #Disable torque.
        self.disable_torque()
        
        #Collection of IDs which experienced error.
        idError = []

        #Iterate through all the motors.
        for id in self.DXL_ID:
            #Send data to modify velocity limit of motors. Receive output whether communication is successful or if hardware error has occured.
            dxlCommunicationResult, dxlHardwareError = self.packet_handler.write4ByteTxRx(self.port_handler, id, self.ADDR_VELOCITY_LIMIT, limit_vel)
            if dxlCommunicationResult != COMM_SUCCESS:
                print(f"{self.packet_handler.getTxRxResult(dxlCommunicationResult)}")
                idError.append(id)
            elif dxlHardwareError != COMM_SUCCESS:
                print(f"{self.packet_handler.getRxPacketError(dxlHardwareError)}")

        #Show status if setting velocity limit has been successful.    
        if len(idError) == 0:
            print(f"Dynamixelmotors has successfully changed the velocity limit to {limit_vel}")
        else:
            print("Error occured on ID: ", end="")
            for id in idError:
                print(f"{id}", end = ', ')

        #Enable torque.
        self.enable_torque()

    def set_all_velocities(self, goal_velocities):
        """
        Set goal velocities for all motors. To be used in velocity control mode.

        Parameters
        ----------
        goal_vel : list[int], required
             List if goal velocity for each motors


        Output
        ----------
        None
        """
        #Check if all goal velocities are within the range
        if any(self.limit_vel < abs(goal_vel) for goal_vel in goal_velocities):
            raise ValueError(f"goal_vel out of bounds!")
        
        #Set goal velocity for each motors
        print(f"setting goal_velocity of all motors to {goal_vel}")
        for index in range(len(self.DXL_ID)):
            #Get the target velocity for this motor.
            goal_vel = goal_velocities[index]

            #Convert the 32-bit velocity value into four bytes for Dynamixel transmission
            param_goal_vel = [DXL_LOBYTE(DXL_LOWORD(goal_vel)), DXL_HIBYTE(DXL_LOWORD(goal_vel)), 
                                 DXL_LOBYTE(DXL_HIWORD(goal_vel)), DXL_HIBYTE(DXL_HIWORD(goal_vel))]

            #Add command into velocity param. Check If there is error in adding parameters.
            dxl_addparam_result = self.velocity_sync_write.addParam(id, param_goal_vel)
            if dxl_addparam_result != True:
                print(f"ID: {id} velocity addparam failed")
        
        #Send all of the commands to all motors. Check if there is communication error.
        dxl_comm_result = self.velocity_sync_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))

        #Clear the parameter
        self.velocity_sync_write.clearParam()

    def set_profile_velocity(self, profile_velocity):
        """
        Sets profile velocity of dynamixel.

        Parameters
        ----------
        profile_velocity: int, required
            Profile velocity of all motors

            
        Output
        ----------
        None
        """
        #Iterate through each motors
        for id in self.DXL_ID:
            #Modify the profile velocity of the current motor. Check if the communication is successful and if there is some error.
            commResult, dxlError = self.packet_handler.write4ByteTxRx(self.port_handler, id, self.ADDR_PRO_PROFILE_VELOCITY, profile_velocity)
            if commResult != COMM_SUCCESS:
                print(f"{self.packet_handler.getTxRxResult(commResult)}")
            elif dxlError != 0:
                print(f"{self.packet_handler.getRxPacketError(dxlError)}")
            else:
                print(f"Profile Velocity of ID: {id} has been updated to {profile_velocity}")
        
        print("\n")

    def set_profile_acceleration(self, profile_acceleration):
        """
        Sets profile acceleration of dynamixel.

        Parameters
        ----------
        profile_acceleration: int, required
            Profile acceleration of all motors

            
        Output
        ----------
        None
        """
        #Iterate through each motors
        for id in self.DXL_ID:
            #Modify the profile velocity of the current motor. Check if the communication is successful and if there is some error.
            commResult, dxlError = self.packet_handler.write4ByteTxRx(self.port_handler, id, self.ADDR_PRO_PROFILE_ACCELARATION, profile_acceleration)
            if commResult != COMM_SUCCESS:
                print(f"{self.packet_handler.getTxRxResult(commResult)}")
            elif dxlError != 0:
                print(f"{self.packet_handler.getRxPacketError(dxlError)}")
            else:
                print(f"Profile Velocity of ID: {id} has been updated to {profile_acceleration}")
        
        print("\n")

    def rotate(self, list_of_goal_positions):
        """
        Rotates all of the motors according to the goal position. 
        Works for position and extended position control.
        Use if there are few motors (less than 10). Otherwise, use modified_rotate().

        Parameters
        ----------
        list_of_goal_positions: list[int], required
            List of goal positions of all motors.

            
        Output
        ----------
        None
        """
        #Check if there is the length of list_of_goal_positions and self.DXL_ID is the same.
        if len(list_of_goal_positions) != len(self.DXL_ID):
            raise ValueError(f"Lengths of list are not matched!")

        #Iterates through all of the motors and goal positions.
        for index in range(len(list_of_goal_positions)):
            print(f"id: {self.DXL_ID[index]}, pos: {list_of_goal_positions[index] }")

            #Adds offset to the final goal_pos
            goal_pos = list_of_goal_positions[index] + self.offset[index]

            #Convert the 32-bit goal_position value into four bytes for Dynamixel transmission.
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_pos)), DXL_HIBYTE(DXL_LOWORD(goal_pos)), 
                                 DXL_LOBYTE(DXL_HIWORD(goal_pos)), DXL_HIBYTE(DXL_HIWORD(goal_pos))]
            
            #Add command into position param. Check If there is error in adding parameters.
            dxl_addparam_result = self.position_sync_write.addParam(self.DXL_ID[index], param_goal_position)
            if dxl_addparam_result != True:
                print(f"ID:{self.DXL_ID[index]} addparam failed")
        
        #Send all of the commands to all motors. Check if there is communication error.
        dxl_comm_result = self.position_sync_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))

        #Clear the parameter.
        self.position_sync_write.clearParam()

    def helper_marker(self, list, divs):
        """
        Helps modified_rotate function by returning a proper division of markers.

        Parameters
        ----------
        list: list[int], required
            List of goal positions of all motors.

        divs: int, required
            Number of division in which the command would be sent.
            
        Output
        ----------
        List of command divided according to divs
        """
        return [0]  + [round(len(list) * (sub + 1)/(divs)) for sub in range(divs)]

    def modified_rotate(self, list_of_goal_positions):
        """
        Rotates all of the motors according to the goal position. 
        Works for position and extended position control mode
        Use for more than 10 dynamixels

        Rotates all of the motors. Works for position and extended position control.
        Works for more than 10 dynamixels by adding a delay for each command according 
        to the set number of divisions.

        Parameters
        ----------
        list_of_goal_positions: list[int], required
            List of goal positions of all motors.

            
        Output
        ----------
        None
        """
        #Check if there the length of goal_positions and motors are the same.
        if len(list_of_goal_positions) != len(self.DXL_ID):
            raise ValueError(f"Lengths of list are not matched!")

        #Gets markers that would divide the goal_positions sending 2 commands at a time separated by 1ms of delay.
        markers = self.helper_marker(list_of_goal_positions, 10)
        markers.reverse()
        delay = 0.001

        print(list_of_goal_positions)

        #Iterates motors of each divisions.
        for marker_index in range(len(markers) - 1):
            #Obtains index of the current division.
            for index in range(markers[marker_index + 1], markers[marker_index]):
                #Add offset to the goal_pos.
                goal_pos = int(list_of_goal_positions[index] + self.offset[index])
                
                #Convert the 32-bit goal_position value into four bytes for Dynamixel transmission.
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_pos)), DXL_HIBYTE(DXL_LOWORD(goal_pos)), 
                                    DXL_LOBYTE(DXL_HIWORD(goal_pos)), DXL_HIBYTE(DXL_HIWORD(goal_pos))]
                
                #Add command into position param. Check If there is error in adding parameters.
                dxl_addparam_result = self.position_sync_write.addParam(self.DXL_ID[index], param_goal_position)
                if dxl_addparam_result != True:
                    print(f"ID:{self.DXL_ID[index]} addparam failed")
            
            #Send all of the commands to all motors. Check if there is communication error.
            dxl_comm_result = self.position_sync_write.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(self.packet_handler.getTxRxResult(dxl_comm_result))

            #Add delay before sending the next command.
            time.sleep(delay)

        #Clear param.
        self.position_sync_write.clearParam()

    
    def torque_rotate(self, list_of_goal_currents):
        """
        Rotates all of the motors according to the goal current. 
        Works for current control.

        Parameters
        ----------
        list_of_goal_currents: list[int], required
            List of goal currents of all motors.

            
        Output
        ----------
        None
        """
        #Check if there the length of goal_positions and motors are the same.
        if len(list_of_goal_currents) != len(self.DXL_ID):
            raise ValueError(f"Lengths of list are not matched!")
        
        #Iterates through each motors
        for index in range(len(list_of_goal_currents)):
            goal_curr = list_of_goal_currents[index]

            #Split the 16-bit goal current value into low and high bytes for Dynamixel transmission
            param_goal_current = [DXL_LOBYTE(DXL_LOWORD(goal_curr)), DXL_HIBYTE(goal_curr)]
            
            #Add command into current param. Check If there is error in adding parameters.
            dxl_addparam_result = self.current_sync_write.addParam(self.DXL_ID[index], param_goal_current)
            if dxl_addparam_result != True:
                print(f"ID:{self.DXL_ID[index]} addparam failed")
        
        #Send all of the commands to all motors. Check if there is communication error.
        dxl_comm_result = self.current_sync_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))
        
        #Clear param.
        self.current_sync_write.clearParam()

    def reboot_all_broadcast(self):
        """
        Reboot all Dynamixels using broadcast ID

        Parameters
        ----------
        None

            
        Output
        ----------
        None
        """
        # reboot all motors
        self.packet_handler.reboot(self.port_handler, 254)

        #Add delay
        time.sleep(1)

        # clear sync buffers
        self.position_sync_read.clearParam()
        self.velocity_sync_read.clearParam()
        self.current_sync_read.clearParam()

        self.position_sync_write.clearParam()
        self.velocity_sync_write.clearParam()
        self.current_sync_write.clearParam()

        # reinitialize motors
        self.initialize(
            self.operating_mode,
            self.profile_velocity,
            self.profile_acceleration
        )    

        
    def close(self):
        """
        Close communication for all dynamixel motors.

        Parameters
        ----------
        None

            
        Output
        ----------
        None
        """
        #Clear sync buffers.
        self.position_sync_read.clearParam()
        self.velocity_sync_read.clearParam()
        self.current_sync_read.clearParam()

        #disable torque of all dynamixel motors.
        self.disable_torque()

        # Read hardware error status
        for id in self.DXL_ID:
            hw_error, comm_result2, dxl_error2 = self.packet_handler.read1ByteTxRx(
                self.port_handler,
                id,
                self.ADDR_PRO_HARDWARE_ERROR_STATUS
            )

            if comm_result2 != COMM_SUCCESS:
                print(f"[ID {id}] HW ERROR READ FAILED: {self.packet_handler.getTxRxResult(comm_result2)}")
                

            if dxl_error2 != 0:
                print(f"[ID {id}] HW PACKET ERROR: {self.packet_handler.getRxPacketError(dxl_error2)}")
                

            if hw_error != 0:
                print(f"[ID {id}] HARDWARE ERROR STATUS: {hw_error}")

                if hw_error & 1:
                    print("  - Input Voltage Error")
                if hw_error & 2:
                    print("  - Overheating Error")
                if hw_error & 4:
                    print("  - Motor Encoder Error")
                if hw_error & 8:
                    print("  - Electrical Shock Error")
                if hw_error & 16:
                    print("  - Overload Error")
            print("========")
        
        #Close port
        self.port_handler.closePort()


