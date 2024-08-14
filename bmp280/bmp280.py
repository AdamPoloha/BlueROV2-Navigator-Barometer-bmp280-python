#!/usr/bin/env python3

# inspiration and help drawn from
# https://github.com/adafruit/Adafruit_CircuitPython_BMP280/blob/master/adafruit_bmp280.py
# see also
# https://github.com/BoschSensortec/BMP280_driver/blob/master/bmp280.c

import smbus2
import time
import struct

_address = 0x76

REG_ID = 0xd0
ID_BMP280 = 0x58

REG_RESET = 0xe0

RESET_COMMAND = 0xb6
REG_STATUS = 0xf3
REG_CTRL_MEAS = 0xf4
REG_CONFIG = 0xf5

REG_COMP_BASE = 0x88
REG_COMP_LENGTH = 24

REG_DATA_BASE = 0xf7
REG_DATA_LENGTH = 6

class Compensation:
    def __init__(self, data):
        default = False
        if not default:
            #print(data)
            datac = [27730, 25970, 50, 37256, -10566, 3024, 6799, -70, -7, 15500, -14600, 6000] #default
            if len(data) != REG_COMP_LENGTH:
                raise Exception("expected %d bytes, got %d", REG_COMP_LENGTH, len(data))
            #("<HhhHhhhhhhhh", data)
            #(27730, 25970, 50, 37256, -10566, 3024, 6799, -70, -7, 15500, -14600, 6000)
            c = 0
            while c < 24:
                h = data[c+1]
                if h > 127 and c != 0 and c != 6:
                    h = h - 256
                datac[int(c/2)] = (h * 256) + data[c]
                #print(datac[int(c/2)])
                c = c + 2
            #exit()
            self.data = datac

            self.T1 = datac[0]
            self.T2 = datac[1]
            self.T3 = datac[2]
            self.P1 = datac[3]
            self.P2 = datac[4]
            self.P3 = datac[5]
            self.P4 = datac[6]
            self.P5 = datac[7]
            self.P6 = datac[8]
            self.P7 = datac[9]
            self.P8 = datac[10]
            self.P9 = datac[11]

class Data:
    def __init__(self, data, compensation):
        if len(data) != REG_DATA_LENGTH:
            raise Exception("expected %d bytes, got %d" % (REG_DATA_LENGTH % len(data)))
        self.data = data
        self.compensation = compensation
        self.pressure_raw = ((data[0] * 65536) + (data[1] * 256) + data[2]) / 16 #int.from_bytes(data[:3], "big") >> 4 # aka / 16
        self.temperature_raw = ((data[3] * 65536) + (data[4] * 256) + data[5]) / 16 #int.from_bytes(data[3:], "big") >> 4 # aka / 16
        self.temperature = Data.calculate_temperature(self.temperature_raw, self.compensation)
        self.pressure = Data.calculate_pressure(self.pressure_raw, self.temperature, self.compensation)

    @staticmethod
    def calculate_temperature(temperature_raw, compensation):
        var1 = (
            temperature_raw / 16384.0 - compensation.T1 / 1024.0
        ) * compensation.T2
        var2 = (
            (temperature_raw / 131072.0 - compensation.T1 / 8192.0)
            * (temperature_raw / 131072.0 - compensation.T1 / 8192.0)
        ) * compensation.T3

        return int(var1 + var2) / 5120.0

    @staticmethod
    def calculate_pressure(pressure_raw, temperature, compensation):

        t_fine = 5120.0 * temperature # todo find a better way to do this
        var1 = float(t_fine) / 2.0 - 64000.0
        var2 = var1 * var1 * compensation.P6 / 32768.0
        var2 = var2 + var1 * compensation.P5 * 2.0
        var2 = var2 / 4.0 + compensation.P4 * 65536.0
        var3 = compensation.P3 * var1 * var1 / 524288.0
        var1 = (var3 + compensation.P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * compensation.P1

        pressure = 1048576.0 - pressure_raw
        pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
        var1 = compensation.P9 * pressure * pressure / 2147483648.0
        var2 = pressure * compensation.P8 / 32768.0
        pressure = pressure + (var1 + var2 + compensation.P7) / 16.0
        pressure /= 100

        return pressure

class BMP280:
    def __init__(self, bus=1, osrs_t=0b010, osrs_p=0b101, mode=0b11, filter=0b100):
        self._bus = smbus2.SMBus(bus)

        self.osrs_t = osrs_t
        self.osrs_p = osrs_p
        self.mode = mode
        self.t_sb = 0
        self.filter = filter
        self.initialize()

    def initialize(self):
        self.reset()

        # give it some time to reset, otherwise compensation readout is garbage
        time.sleep(0.010)

        self.id = self.read(REG_ID)[0]
        if self.id != ID_BMP280:
            raise Exception("device is not a BMP280. got id %.2x, expected %.2x" % (self.id, ID_BMP280))

        # todo reset
        self.compensation = self.get_compensation()

        self.write_config()
        self.write_ctrl()

        # give the sensor some time to stabilize before first measurement
        # no mention of this being required in the datasheet (that i could find)
        # empirically determined that first samples are garbage unless this
        # delay is applied
        time.sleep(0.1)

        return True

    def write_ctrl(self):
        data = self.osrs_t << 5
        data |= self.osrs_p << 2
        data |= self.mode
        self.write(REG_CTRL_MEAS, [data])

    def write_config(self):
        data = self.t_sb << 5
        data |= self.filter << 2
        self.write(REG_CONFIG, [data])

    def reset(self):
        self.write(REG_RESET, [RESET_COMMAND])
    
    def read_id(self):
        return self.read(REG_ID)

    def get_compensation(self):
        compensation_data = self.read(REG_COMP_BASE, REG_COMP_LENGTH)
        return Compensation(compensation_data)

    def get_data(self):
        data = self.read(REG_DATA_BASE, REG_DATA_LENGTH)
        return Data(data, self.compensation)

    def read(self, register, length=1):
        data = self._bus.read_i2c_block_data(_address, register, length)
        return data

    def write(self, register_address, data):
        data.insert(0, register_address)
        msg = smbus2.i2c_msg.write(_address, data)
        self._bus.i2c_rdwr(msg)
        
    def close(self):
        self._bus.close()
