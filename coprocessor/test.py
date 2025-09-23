#!/usr/bin/env python3

from typing import Tuple
from smbus2 import SMBus
from serial import Serial
import RPi.GPIO as GPIO
import time
import struct

I2C_SENSOR_ADDR = 0x17
I2C_RESET_OFFSET = 0x07
I2C_CALIBRATE_OFFSET = 0x06
I2C_POSITION_OFFSET = 0x20
I2C_VELOCITY_OFFSET = 0x26
I2C_ACCELERATION_OFFSET = 0x2C

GPIO_RS485_ENABALE_PINS = [11, 12]
GPIO_RS485_RX_STATE = GPIO.LOW
GPIO_RS485_TX_STATE = GPIO.HIGH

I16_MAX  = ((2 ** 16) // 2) - 1
OTOS_POS_LINEAR_RANGE  = 10     # meters
OTOS_POS_ANGULAR_RANGE = 180    # degrees
OTOS_VEL_LINEAR_RANGE  = 5      # meters/second
OTOS_VEL_ANGULAR_RANGE = 2000   # degrees/second
OTOS_ACC_LINEAR_RANGE  = 157    # meters/second/second
OTOS_ACC_ANGULAR_RANGE = 180000 # degrees/second/second

# Configure GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_RS485_ENABALE_PINS, GPIO.OUT)
GPIO.output(GPIO_RS485_ENABALE_PINS, GPIO_RS485_RX_STATE)

class OtosSensor:
    def __init__(self):
        self.bus = SMBus(1)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        self.bus.close()

    def calibrate(self):
        self.bus.write_byte_data(I2C_SENSOR_ADDR, I2C_CALIBRATE_OFFSET, 0xFF) # Use 255 samples to calibrate
        while self.bus.read_byte_data(I2C_SENSOR_ADDR, I2C_CALIBRATE_OFFSET) != 0x00:
            time.sleep(0.25)

        self.bus.write_byte_data(I2C_SENSOR_ADDR, I2C_RESET_OFFSET, 0x01) # Reset tracking

    def get_position(self) -> Tuple[int, int, int]:
        block = self.bus.read_i2c_block_data(I2C_SENSOR_ADDR, I2C_POSITION_OFFSET, 6)

        x, y, h = struct.unpack("<3h", bytes(block))

        return (
            x * (OTOS_POS_LINEAR_RANGE / I16_MAX),
            y * (OTOS_POS_LINEAR_RANGE / I16_MAX),
            h * (OTOS_POS_ANGULAR_RANGE / I16_MAX)
        )

    def get_velocity(self) -> Tuple[int, int, int]:
        block = self.bus.read_i2c_block_data(I2C_SENSOR_ADDR, I2C_POSITION_OFFSET, 6)

        x, y, h = struct.unpack("<3h", bytes(block))

        return (
            x * (OTOS_VEL_LINEAR_RANGE / I16_MAX),
            y * (OTOS_VEL_LINEAR_RANGE / I16_MAX),
            h * (OTOS_VEL_ANGULAR_RANGE / I16_MAX)
        )

class SmartPortSerial:
    def __init__(self):
        self.serial = Serial("/dev/ttyAMA0", 921600, timeout=1)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        self.serial.close()


# https://github.com/sparkfun/SparkFun_Optical_Tracking_Odometry_Sensor/blob/main/Firmware/OTOS_Register_Map.pdf
with (
    OtosSensor() as otos,
    SmartPortSerial(),
):
    # Calibrate sensor
    print("Calibrating sensor...")
    otos.calibrate()
    print("Sensor calibrated!")

    # Start
