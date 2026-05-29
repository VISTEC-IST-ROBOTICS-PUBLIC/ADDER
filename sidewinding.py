"""
This code is called to let ADDER perform the sidewinding gait.
The file can be called directly to perform a default sidewinding gait.
By adding 1 argument, the duration of the gait can be modified.
If more than 1 argument is specified, all gait parameters can be modified.
At the end of the gait, the collected data is uploaded to a folder for post processing.
"""

from SnakeController import SnakeController
import numpy as np
import pandas as pd
import sys
import os

def sidewinding(amplitude_tangent = 30, angular_frequency_tangent = np.pi, phase_shift_tangent = np.pi/6, 
                    amplitude_normal = 60, angular_frequency_normal = np.pi, phase_shift_normal = np.pi/6, 
                    stopping_constant = 100, profile_velocity = 0, profile_acceleration = 0):
    """
    This method calls the sidewinding gait which shape is modified by its parameters. 
    """
    #Dynamixel IDs.
    id = [x for x in range(20)]

    #Initialize controller
    controller = SnakeController(id, profile_acceleration=profile_acceleration, profile_velocity=profile_velocity)

    #Call the sidewinding gait.
    controller.sidewinding(amplitude_tangent, angular_frequency_tangent, phase_shift_tangent,
                           amplitude_normal, angular_frequency_normal, phase_shift_normal, stopping_constant)
    
    #Close the controller and collect the data.
    time_data, ina_data, mpu_data = controller.rest()

    #Convert them to numpy
    time = np.array(time_data)
    pow  = np.array([x['power'] for x in ina_data])
    acc  = np.array([x['acc'] for x in mpu_data])

    #Compile them in pandas
    df = pd.DataFrame(acc, columns =["ax", "ay", "az"])
    df['power'] = pow
    df['time']  = time

    #Prepare folder names
    folder  =  "sidewinding_data"
    filename = "sidewinding_test.csv"

    #Make the folder if they do not exist
    os.makedirs(folder, exist_ok=True)

    #Write if the name doesnt exist yet
    base, ext = os.path.splitext(filename)
    path = os.path.join(folder, filename)

    #If it does, add a number and increment.
    i = 1
    while os.path.exists(path):
        path = os.path.join(folder, f"{base}_{i}{ext}")
        i += 1

    #Convert pandas to csv and upload to file path
    df.to_csv(path, index=False)

if __name__ == '__main__':
    #If there is one args, it is assumed to be the stopping constant
    if len(sys.argv) == 2:
        stopping_constant = int(sys.argv[1])
        sidewinding(stopping_constant = stopping_constant)

    #If there's more, they are assumed to be the parameters
    elif len(sys.argv) > 1:
        amplitude_tangent = float(sys.argv[1])
        angular_frequency_tangent = float(sys.argv[2])
        phase_shift_tangent = float(sys.argv[3])

        amplitude_normal = float(sys.argv[4])
        angular_frequency_normal = float(sys.argv[5])
        phase_shift_normal = float(sys.argv[6])


        stopping_constant = int(sys.argv[7])
        profile_velocity = int(sys.argv[8])
        profile_acceleration = int(sys.argv[9])


        sidewinding(amplitude_tangent = amplitude_tangent, angular_frequency_tangent = angular_frequency_tangent, phase_shift_tangent = phase_shift_tangent, 
                        amplitude_normal = amplitude_normal, angular_frequency_normal = angular_frequency_normal, phase_shift_normal = phase_shift_normal, 
                        stopping_constant = stopping_constant, profile_velocity = profile_velocity, profile_acceleration =profile_acceleration)
    
    #If there is none, run according to default parameters
    else:
        sidewinding()
