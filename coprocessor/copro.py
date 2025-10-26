#!/usr/bin/env python3

from smbus2 import SMBus
from serial import Serial
from cobs import cobs
import RPi.GPIO as GPIO
import time

I2C_SENSOR_ADDR = 0x17
I2C_RESET_OFFSET = 0x07
I2C_CALIBRATE_OFFSET = 0x06
I2C_OFFSET_OFFSET = 0x10
I2C_SCALAR_OFFSET = 0x04
I2C_POSITION_OFFSET = 0x20
I2C_VELOCITY_OFFSET = 0x26
I2C_ACCELERATION_OFFSET = 0x2C

GPIO_RS485_ENABALE_PINS = [11, 12]
GPIO_RS485_RX_STATE = GPIO.LOW
GPIO_RS485_TX_STATE = GPIO.HIGH

# Configure GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_RS485_ENABALE_PINS, GPIO.OUT)
GPIO.output(GPIO_RS485_ENABALE_PINS, GPIO_RS485_RX_STATE) # Default to recieving

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

    def set_offset(self, block):
        self.bus.write_i2c_block_data(I2C_SENSOR_ADDR, I2C_OFFSET_OFFSET, block)

    def set_scalar(self,block):
        self.bus.write_i2c_block_data(I2C_SENSOR_ADDR, I2C_SCALAR_OFFSET, block)

    def get_position(self) -> bytes:
        block = self.bus.read_i2c_block_data(I2C_SENSOR_ADDR, I2C_POSITION_OFFSET, 6)

        return bytes(block)

    def get_velocity(self) -> bytes:
        block = self.bus.read_i2c_block_data(I2C_SENSOR_ADDR, I2C_VELOCITY_OFFSET, 6)

        return bytes(block)

class SmartPortSerial:
    def __init__(self):
        self.serial = Serial("/dev/ttyAMA0", 921600, timeout=1)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        self.serial.close()

    def send(self,data):
        GPIO.output(GPIO_RS485_ENABALE_PINS, GPIO_RS485_TX_STATE)
        self.serial.write(cobs.encode(data))
        self.serial.write(b'\0')
        self.serial.flush()
        GPIO.output(GPIO_RS485_ENABALE_PINS, GPIO_RS485_RX_STATE)

    def read_byte(self) -> int | None:
        out = self.serial.read()
        if len(out) > 0:
            return out[0]
        else:
            return None


# https://github.com/sparkfun/SparkFun_Optical_Tracking_Odometry_Sensor/blob/main/Firmware/OTOS_Register_Map.pdf
with (
    OtosSensor() as otos,
    SmartPortSerial() as smart_port,
):
    # Calibrate sensor
    print("Calibrating sensor...")
    otos.calibrate()
    print("Sensor calibrated!")

    buffer = []

    while True:
        next_byte = smart_port.read_byte()
        #if next_byte != None:
        #    print(next_byte, chr(next_byte))

        if next_byte == 0:
            try:
                decoded = cobs.decode(bytes(buffer))
            except:
                print("Invalid COBS!")
                continue
            finally:
                buffer = []

            if len(decoded) <= 0:
                continue

            comand = decoded[0]

            match chr(decoded[0]):
                case 'p':
                    print("Sent position")
                    smart_port.send(otos.get_position())

                case 'v': # Get velocity
                    print("Set velocity")
                    smart_port.send(otos.get_velocity())

                case 'c': # Calibrate
                    otos.calibrate()
                    smart_port.send(b'd')

                case 'o': # Set offsets
                    otos.set_offset(decoded[1:7])
                    smart_port.send(b'd')

                case 's': # Set scalars
                    otos.set_scalar(decoded[1:3])
                    smart_port.send(b'd')

                case 'a': # Ping
                    smart_port.send(b'a')

                    print("alive")
        elif next_byte == None:
            continue
        elif len(buffer) < 2048:
            buffer.append(next_byte)
        else:
            buffer = [next_byte]
