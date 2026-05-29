"""
This class obtains sensor data from INA219 and MPU6050 through I2C communication.
The obtained data is then used for post-processing and evaluation.
"""
import time
import struct
from smbus2 import SMBus


class I2CSensorCollector:
    def __init__(self, bus_id=1,
                 mpu_addr=0x68,
                 ina_addr=0x40):
        """
        This class collects data from both MPU6050 and INA219 sensors over I2C.

        Parameters
        ----------
        bus_id : int, optional
            I2C bus number used to communicate with the sensors.
        mpu_addr : int, optional
            I2C address of the MPU6050 sensor.
        ina_addr : int, optional
            I2C address of the INA219 sensor.

        Returns
        -------
        None
        """
        #Initialize SMBus to read data through I2C.
        self.bus = SMBus(bus_id)

        #Initialize mpu and ina address
        self.mpu_addr = mpu_addr
        self.ina_addr = ina_addr

        # Initialize devices
        self._init_mpu6050()
        self._init_ina219()

        # Initialize zero offsets first
        self.ax_0 = 0
        self.ay_0 = 0
        self.az_0 = 0

        # Calibrate accelerometer zero
        self._calibrate_acc()

    # ==========================================================
    # ---------------- MPU6050 (GY-521) ------------------------
    # ==========================================================

    def _init_mpu6050(self):
        self.bus.write_byte_data(self.mpu_addr, 0x6B, 0x00)  # Wake up
        self.bus.write_byte_data(self.mpu_addr, 0x1C, 0x00)  # ±2g
        self.bus.write_byte_data(self.mpu_addr, 0x1B, 0x00)  # ±250°/s
        time.sleep(0.1)

    def _read_word(self, addr, reg):
        """
        Read a signed 16-bit value from two consecutive I2C registers.

        Parameters
        ----------
        addr : int
            I2C address of the target device.
        reg : int
            Starting register address.

        Returns
        -------
        int
            Signed 16-bit integer value read from the device.
        """
        high = self.bus.read_byte_data(addr, reg)
        low = self.bus.read_byte_data(addr, reg + 1)
        value = (high << 8) + low

        if value >= 0x8000:
            value -= 65536
        return value

    def _read_mpu_raw(self):
        """
        Read raw accelerometer and gyroscope measurements from the MPU6050.

        Accelerometer values are returned in units of g and gyroscope
        values are returned in degrees per second.

        Parameters
        ----------
        None

        Returns
        -------
        tuple
            (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
        """
        acc_x = self._read_word(self.mpu_addr, 0x3B) / 16384.0
        acc_y = self._read_word(self.mpu_addr, 0x3D) / 16384.0
        acc_z = self._read_word(self.mpu_addr, 0x3F) / 16384.0

        gyro_x = self._read_word(self.mpu_addr, 0x43) / 131.0
        gyro_y = self._read_word(self.mpu_addr, 0x45) / 131.0
        gyro_z = self._read_word(self.mpu_addr, 0x47) / 131.0

        return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z

    def _calibrate_acc(self, samples=200):
        """
        Estimate accelerometer bias offsets while the sensor is stationary.

        The sensor should be placed flat and motionless with the Z-axis
        pointing upward during calibration.

        Parameters
        ----------
        samples : int, optional
            Number of samples used to estimate the bias.

        Returns
        -------
        None
        """

        print("Calibrating accelerometer...")
        print("Keep sensor flat and still.")

        ax_sum = ay_sum = az_sum = 0.0

        for _ in range(samples):
            ax, ay, az, _, _, _ = self._read_mpu_raw()
            ax_sum += ax
            ay_sum += ay
            az_sum += az
            time.sleep(0.01)

        ax_mean = ax_sum / samples
        ay_mean = ay_sum / samples
        az_mean = az_sum / samples

        # Expected values when flat
        expected_x = 0.0
        expected_y = 0.0
        expected_z = 1.0  # gravity

        self.ax_0 = ax_mean - expected_x
        self.ay_0 = ay_mean - expected_y
        self.az_0 = az_mean - expected_z

        print("Calibration done.")
        print("Biases:")
        print("ax bias:", self.ax_0)
        print("ay bias:", self.ay_0)
        print("az bias:", self.az_0)

    def read_mpu6050(self):
        """
        Read calibrated accelerometer and gyroscope data from the MPU6050.

        Accelerometer readings are corrected using the previously
        calculated calibration offsets.

        Parameters
        ----------
        None

        Returns
        -------
        dict
            Dictionary containing:
            - "acc": (acc_x, acc_y, acc_z) in g
            - "gyro": (gyro_x, gyro_y, gyro_z) in deg/s
        """
        ax, ay, az, gx, gy, gz = self._read_mpu_raw()

        acc_x = ax - self.ax_0
        acc_y = ay - self.ay_0
        acc_z = az - self.az_0

        return {
            "acc": (acc_x, acc_y, acc_z),
            "gyro": (gx, gy, gz)
        }

    # ==========================================================
    # -------------------- INA219 ------------------------------
    # ==========================================================

    def _init_ina219(self):
        """
        Configure and initialize the INA219 current and voltage sensor.

        Sets the operating range, ADC resolution, and calibration
        register values.

        Parameters
        ----------
        None

        Returns
        -------
        None
    """
        # Configuration register (0x00)
        # 32V range, 320mV gain, 12-bit ADC
        config = 0x019F
        self.bus.write_word_data(self.ina_addr, 0x00, self._swap_bytes(config))

        # Calibration register (example for 0.1Ω shunt, max 3.2A)
        calibration = 4096
        self.bus.write_word_data(self.ina_addr, 0x05, self._swap_bytes(calibration))

    def _swap_bytes(self, value):
        """
        Swap the byte order of a 16-bit value.

        This is used because the INA219 expects register values in a
        different byte order than the host system.

        Parameters
        ----------
        value : int
            16-bit integer value.

        Returns
        -------
        int
            Byte-swapped 16-bit integer.
        """
        return struct.unpack("<H", struct.pack(">H", value))[0]

    def read_ina219(self):
        """
        Read voltage, current, and power measurements from the INA219.

        Parameters
        ----------
        None

        Returns
        -------
        dict
            Dictionary containing:
            - "voltage": Bus voltage in volts (V)
            - "current": Current in amperes (A)
            - "power": Power in watts (W)
        """
        # Read bus voltage register (0x02)
        data = self.bus.read_i2c_block_data(self.ina_addr, 0x02, 2)
        raw_bus = (data[0] << 8) | data[1]

        bus_voltage = (raw_bus >> 3) * 0.004  # 4mV per bit

        # Read current register (0x04)
        data = self.bus.read_i2c_block_data(self.ina_addr, 0x04, 2)
        raw_current = (data[0] << 8) | data[1]

        if raw_current > 32767:
            raw_current -= 65536

        current = raw_current * 0.0001  # 100uA LSB

        # Read power register (0x03)
        data = self.bus.read_i2c_block_data(self.ina_addr, 0x03, 2)
        raw_power = (data[0] << 8) | data[1]

        power = raw_power * 0.002  # 20 × Current_LSB

        return {
            "voltage": bus_voltage,
            "current": current,
            "power": power
        }

    # ==========================================================
    # -------------------- Unified Read ------------------------
    # ==========================================================

    def read_all(self):
        """
        Read data from all connected sensors.

        Parameters
        ----------
        None

        Returns
        -------
        dict
            Dictionary containing measurements from both MPU6050
            and INA219 sensors.
        """
        data = {}
        data["mpu6050"] = self.read_mpu6050()
        data["ina219"] = self.read_ina219()
        return data

    def close(self):
        """
        Close the I2C bus connection.

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        self.bus.close()

