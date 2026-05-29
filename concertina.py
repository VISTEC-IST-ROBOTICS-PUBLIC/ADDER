"""
This code is called to let ADDER perform the concertina gait.
The file can be called directly to perform a default concertina gait.
By adding 1 argument, the duration of the gait can be modified.
If more than 1 argument is specified, all gait parameters can be modified.
At the end of the gait, the collected data is uploaded to a folder for post processing.
"""

from SnakeController import SnakeController
import numpy as np
import pandas as pd
import sys
import os

def concertina(profile_velocity=0, profile_acceleration=0, stopping_constant = 10, number_of_transitions = 1):
    """
    This method calls the concertina gait modified by the parameters. 
    """
    #Dynamixel IDs.
    id = [x for x in range(20)]

    #Initialize controller.
    controller = SnakeController(id, profile_acceleration=profile_acceleration, profile_velocity=profile_velocity)
    
    #Call the serpentine gait.
    controller.concertina(number_of_commands= stopping_constant, number_of_transitions = number_of_transitions)

    #Close the controller and collect the data.
    time_data, ina_data, mpu_data = controller.rest()

    #Convert them to numpy.
    time = np.array(time_data)
    pow  = np.array([x['power'] for x in ina_data])
    acc  = np.array([x['acc'] for x in mpu_data])

    #compile them in pandas
    df = pd.DataFrame(acc, columns =["ax", "ay", "az"])
    df['power'] = pow
    df['time']  = time

    #Prepare folder names
    folder   = "concertina_data"
    filename = "concertina_test.csv"

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
        concertina(stopping_constant = stopping_constant)

    #If there's more, they are assumed to be the parameters
    elif len(sys.argv) > 1:
        profile_velocity = int(sys.argv[1])
        profile_acceleration = int(sys.argv[2])
        stopping_constant = int(sys.argv[3])
        number_of_transitions = int(sys.argv[4])

        concertina(profile_velocity, profile_acceleration, stopping_constant, number_of_transitions)

    #If there is none, run according to default parameters
    else:
        concertina()
