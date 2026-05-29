from Sensors import I2CSensorCollector
import time 
import numpy as np
import pandas as pd

class SensorLogger:
    def __init__(self):
        self.sensor = I2CSensorCollector()
    
    def record_data(self, duration = 10, rate = 100):
        start = time.monotonic()
        period = 1/rate

        acc_data = []
        pwr_data = []
        time_data = []

        while True:
            loop_start = time.monotonic()
            curr = loop_start - start
            if curr > duration:
                break

            acc_data.append(self.sensor.read_mpu6050()["acc"])
            pwr_data.append(self.sensor.read_ina219()["power"])
            time_data.append(curr)

            time.sleep(max(0, period - (time.monotonic() - loop_start)))

        acc = np.array(acc_data)
        pwr = np.array(pwr_data)

        df = pd.DataFrame(acc, columns =["ax", "ay", "az"])
        df["pwr"] = pwr
        df["time"] = time_data

        df.to_csv("sensor_test.csv", index=False)

if __name__ == '__main__':
    logger = SensorLogger()

    print("Starting...")
    logger.record_data()
        

        


