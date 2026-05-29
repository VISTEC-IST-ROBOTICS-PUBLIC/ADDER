from Sensors import I2CSensorCollector

if __name__ == "__main__":
    sensor = I2CSensorCollector()
    voltage = sensor.read_ina219()['voltage']
    print(f"Voltage: {voltage}V")