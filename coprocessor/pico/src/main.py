import hashlib
import os
import struct
import sys
import time
from typing import Final, cast

import machine
from micropython import const

import cobs
from blinker import PioBlinker
from otos import OtosSensor
from ws2812b import PioWS2812B

def sm_id(t: tuple[int, int]) -> int:
    return t[0] * 4 + t[1]

TEST_MODE = __debug__

WS2812B_PIO = (0, 0)
BLINKER_PIO = (1, 0)

RS485_EN_PIN = "GP11"
RS485_TX_PIN = "GP12"
RS485_RX_PIN = "GP13"
ADDR_LED_PIN = "GP10"
OTOS_SDA_PIN = "GP8"
OTOS_SCL_PIN = "GP9"
STATUS_LED_PIN = "GP2"

RS485_UART = machine.UART(
    0, baudrate=921600, tx=machine.Pin(RS485_TX_PIN), rx=machine.Pin(RS485_RX_PIN)
)
RS485_EN_OUT = machine.Pin(RS485_EN_PIN, machine.Pin.OUT)
ADDR_LED_OUT = machine.Pin(ADDR_LED_PIN, machine.Pin.OUT)
STATUS_LED_OUT = machine.Pin(STATUS_LED_PIN, machine.Pin.OUT)
OTOS_I2C = machine.I2C(
    sda=machine.Pin(OTOS_SDA_PIN),
    scl=machine.Pin(OTOS_SCL_PIN),
    freq=1000000,  # 1MHz, Sparkfun calibration code suggests this speed is supported
)

LED_BLACK = const(0)
LED_RED = const(1)
LED_BLUE = const(2)
LED_RAINBOW = const(3)
LED_ROTATE = const(4)
LED_GREEN = const(5)
LED_WHITE = const(6)
LED_ORANGE = const(7)


class RGB:
    length: Final[int]
    brightness: float
    mode: int
    n: Final[PioWS2812B]

    def __init__(self, pin: machine.Pin, length: int, brightness: float):
        self.length = length
        self.n = PioWS2812B(WS2812B_PIO, pin, length)
        self.brightness = brightness
        self.set_mode(LED_BLACK)

    def adjust_brightness(self, rgb: tuple[int, int, int]) -> tuple[int, int, int]:
        return (
            int(rgb[0] * self.brightness),
            int(rgb[1] * self.brightness),
            int(rgb[2] * self.brightness),
        )

    def set_mode(self, mode: int):
        self.mode = mode
        self.update()

    def update(self):
        if self.mode == LED_BLACK:
            for i in range(self.length):
                self.n[i] = self.adjust_brightness((0, 0, 0))

        elif self.mode == LED_RED:
            for i in range(self.length):
                self.n[i] = self.adjust_brightness((255, 0, 0))

        elif self.mode == LED_BLUE:
            for i in range(self.length):
                self.n[i] = self.adjust_brightness((0, 0, 255))

        elif self.mode == LED_RAINBOW:
            for i in range(self.length):
                pos = int((i * 256) / self.length) % 256
                if pos < 85:
                    c = (
                        pos * 3,
                        255 - pos * 3,
                        0,
                    )
                elif pos < 170:
                    pos -= 85
                    c = (
                        255 - pos * 3,
                        0,
                        pos * 3,
                    )
                else:
                    pos -= 170
                    c = (
                        0,
                        pos * 3,
                        255 - pos * 3,
                    )
                self.n[i] = self.adjust_brightness(c)
            self.mode = LED_ROTATE

        elif self.mode == LED_ROTATE:
            last = self.n[0]
            for i in range(self.length - 1):
                self.n[i] = self.n[i + 1]
            self.n[-1] = last

        elif self.mode == LED_GREEN:
            for i in range(self.length):
                self.n[i] = self.adjust_brightness((0, 255, 0))

        elif self.mode == LED_WHITE:
            for i in range(self.length):
                self.n[i] = self.adjust_brightness((255, 255, 255))

        elif self.mode == LED_ORANGE:
            for i in range(self.length):
                self.n[i] = self.adjust_brightness((255, 50, 0))

        self.n.write()


class VexBrain:
    uart: Final[machine.UART]
    enable_pin: Final[machine.Pin]
    buffer: bytearray

    def __init__(self, uart: machine.UART, enable_pin: machine.Pin):
        self.uart = uart
        self.enable_pin = enable_pin
        self.buffer = bytearray()

    def send(self, data: bytes):
        self.enable_pin.value(1)
        _ = self.uart.write(cobs.encode(data))
        self.uart.flush()
        self.enable_pin.value(0)

    def receive(self):
        for _ in range(self.uart.any()):
            byte = cast(bytes, self.uart.read(1))[0]
            self.buffer.append(byte)
            if self.buffer[-1] == 0x00:
                encoded = bytes(self.buffer)
                self.buffer = bytearray()
                return cobs.decode(encoded)
        return None


def main():
    STATUS_LED = PioBlinker(BLINKER_PIO, STATUS_LED_OUT)
    LED = RGB(ADDR_LED_OUT, 15, 0.75)

    brain = VexBrain(RS485_UART, RS485_EN_OUT)

    # Wait till otos connected
    STATUS_LED.blink(2)
    LED.set_mode(LED_RED)
    otos = OtosSensor(OTOS_I2C)
    otos_attempt = 1
    while True:
        otos_versions = otos.get_versions()
        if otos_versions is not None:
            print(
                f"OTOS sensor connected: hardware {otos_versions[0]} firmware {otos_versions[1]}"
            )
            break
        else:
            print(
                f"WARNING: OTOS sensor not connected, trying again (attempt {otos_attempt})..."
            )
            otos_attempt += 1

            time.sleep(1)

    LED.set_mode(LED_ORANGE)
    otos.calibrate()

    LED.set_mode(LED_GREEN)
    STATUS_LED.blink(1)  # Only blink once to show success
    led_mode = 0
    i = 0
    while True:
        if __debug__ and i % 1000 == 0:
            pos = struct.unpack("<3h", otos.get_position())
            print(
                f"X: {pos[0] / 32767 * 10}m, Y: {pos[1] / 32767 * 10}m, H: {pos[2] / 32767 * 180}deg"
            )

        # TODO: UART RX_IDLE interrupt, preferably hardware for ultimate speedy communication
        data = brain.receive()
        if data is not None:
            if data[0:1] == b"p":
                brain.send(otos.get_position())
            elif data[0:1] == b"P":
                otos.set_position(data[1:])
                brain.send(b"d")
            elif data[0:1] == b"v":
                brain.send(otos.get_velocity())
            elif data[0:1] == b"c":
                otos.calibrate()
                brain.send(b"d")
            elif data[0:1] == b"o":
                otos.set_offset(data[1:])
                brain.send(b"d")
            elif data[0:1] == b"s":
                otos.set_scalar(data[1:])
                brain.send(b"d")
            elif data[0:1] == b"S":
                brain.send(otos.get_stddev())
            elif data[0:1] == b"a":  # ping/hash
                # start_ms = time.ticks_ms()
                hash = hashlib.sha256()
                files = os.listdir()
                files.sort()
                for filename in files:
                    if not filename.endswith(".py"):
                        continue

                    hash.update(filename.encode())

                    with open(filename, "rb") as file: # pyright: ignore[reportUnknownVariableType]
                        hash.update(cast(bytes, file.read()))

                digest = cast(bytes, hash.digest())
                brain.send(digest)

                # end_ms = time.ticks_ms()
                # print("Took", end_ms - start_ms, "ms to calculate hash")
            elif data[0:1] == b"l":
                led_mode = (led_mode + 1) % 6
                LED.set_mode(led_mode)
                brain.send(b"d")
        if i % 25 == 0:
            LED.update()
        i += 1

        time.sleep(0)  # Yield to scheduler


if __name__ == "__main__":
    while True:
        try:
            main()
        except Exception as e:
            sys.print_exception(e)
            time.sleep(1)
