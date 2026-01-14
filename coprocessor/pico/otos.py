import time
import struct
from typing import Optional, Tuple

OTOS_PRODUCT_ID = 0x5F

I2C_SENSOR_ADDR = 0x17
I2C_PRODUCT_ID_ADDR = 0x00
I2C_RESET_OFFSET = 0x07
I2C_CALIBRATE_OFFSET = 0x06
I2C_OFFSET_OFFSET = 0x10
I2C_SCALAR_OFFSET = 0x04
I2C_POSITION_OFFSET = 0x20
I2C_VELOCITY_OFFSET = 0x26
I2C_ACCELERATION_OFFSET = 0x2C

class OtosSensor:
    def __init__(self, i2c):
        self.i2c = i2c

    def get_versions(self) -> Optional[Tuple[str, str]]:
        try:
            # First byte is 0x5F product ID
            # Second and third bytes are 2 nibbles each (major, minor) of hardware and firmware versions respectively
            product_info = self.i2c.readfrom_mem(I2C_SENSOR_ADDR, I2C_PRODUCT_ID_ADDR, 3)
        except OSError:
            return None # Not connected/no response

        if product_info[0] != 0x5F: # Invalid product ID response, likely not OTOS
            return None

        [hardware, firmware] = struct.unpack("<2b", product_info[1:])

        return (f"v{hardware >> 4}.{hardware & 0b1111}", f"v{firmware >> 4}.{firmware & 0b1111}")

    def calibrate(self):
        self.i2c.writeto_mem(I2C_SENSOR_ADDR, I2C_CALIBRATE_OFFSET, b'\xFF') # Use 255 samples to calibrate
        time.sleep(0.003) # Wait to ensure write

        while self.i2c.readfrom_mem(I2C_SENSOR_ADDR, I2C_CALIBRATE_OFFSET, 1)[0] != 0x00:
            time.sleep(0.1)

        self.i2c.writeto_mem(I2C_SENSOR_ADDR, I2C_RESET_OFFSET, b'\x01') # Reset tracking

    def set_offset(self, block: bytes):
        self.i2c.writeto_mem(I2C_SENSOR_ADDR, I2C_OFFSET_OFFSET, block)

    def set_scalar(self, block: bytes):
        self.i2c.writeto_mem(I2C_SENSOR_ADDR, I2C_SCALAR_OFFSET, block)

    def get_position(self) -> bytes:
        block = self.i2c.readfrom_mem(I2C_SENSOR_ADDR, I2C_POSITION_OFFSET, 6)

        return bytes(block)

    def set_position(self, block: bytes):
        self.i2c.writeto_mem(I2C_SENSOR_ADDR, I2C_POSITION_OFFSET, block)

    def get_velocity(self) -> bytes:
        block = self.i2c.readfrom_mem(I2C_SENSOR_ADDR, I2C_VELOCITY_OFFSET, 6)

        return bytes(block)
