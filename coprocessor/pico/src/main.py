TEST_MODE = const(False)

import hashlib
import os
import struct
import sys
import micropython
import time
from typing import Final, cast

import machine
from micropython import RingIO, const
micropython.opt_level(0 if TEST_MODE else 3)

import cobs
from blinker import PioBlinker
from otos import OtosSensor
from ws2812b import PioWS2812B
from consts import *



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

class RGB:
    length: Final[int]
    brightness: int
    mode: int
    n: Final[PioWS2812B]

    MODE_STATIC: int = const(1)
    MODE_ROTATE: int = const(2)

    BLACK: int = const(0x000000)
    RED: int = const(0xFF0000)
    GREEN: int = const(0x00FF00)
    BLUE: int = const(0x0000FF)
    WHITE: int = const(0xFFFFFF)
    ORANGE: int = const(0xFF3200)

    def __init__(self, pin: machine.Pin, length: int, brightness: int):
        self.length = length
        self.n = PioWS2812B(WS2812B_PIO, pin, length)
        self.brightness = brightness
        self.set_mode(self.MODE_STATIC)

    @micropython.viper
    def adjust_brightness(self, color: int, brightness: int) -> int:
        return (
            (((color & 0xFF0000) * brightness // 100) & 0xFF0000)
            | (((color & 0xFF00) * brightness // 100) & 0xFF00)
            | (((color & 0xFF) * brightness // 100) & 0xFF)
        )

    def set_mode(self, mode: int):
        self.mode = mode
        self.update()

    @micropython.native
    def update(self):
        if self.mode == RGB.MODE_STATIC:
            pass

        elif self.mode == RGB.MODE_ROTATE:
            last = self.n[0]
            for i in range(self.length - 1):
                self.n[i] = self.n[i + 1]
            self.n[-1] = last

        self.n.write()

    @micropython.native
    def set_color(self, color: int):
        color = self.adjust_brightness(color, self.brightness)
        for i in range(self.length):
            self.n[i] = color

        self.n.write()
        self.mode = RGB.MODE_STATIC

    @micropython.native
    def set_rainbow(self):
        for i in range(self.length):
            pos = (i * 256 // self.length) % 256
            if pos < 85:
                c = (
                    ((pos * 3) << 16)
                    | ((255 - pos * 3) << 8)
                )
            elif pos < 170:
                pos -= 85
                c = (
                    ((255 - pos * 3) << 16)
                    | (pos * 3)
                )
            else:
                pos -= 170
                c = (
                    ((pos * 3) << 8)
                    | (255 - pos * 3)
                )
            self.n[i] = self.adjust_brightness(c, self.brightness)

        self.n.write()


class VexBrain:
    uart: Final[machine.UART]
    enable_pin: Final[machine.Pin]
    buffer: RingIO

    MTU: int = const(1024)

    def __init__(self, uart: machine.UART, enable_pin: machine.Pin):
        self.uart = uart
        self.enable_pin = enable_pin
        self.buffer = RingIO(VexBrain.MTU)

    def send(self, data: bytes):
        self.enable_pin.value(1)
        _ = self.uart.write(cobs.encode(data))
        self.uart.flush()
        self.enable_pin.value(0)

    def receive(self) -> bytes | None:
        for _ in range(self.uart.any()):
            byte = cast(bytes, self.uart.read(1))
            _ = self.buffer.write(byte)
            if byte[0] == 0x00:
                encoded = self.buffer.read()
                return cobs.decode(encoded)

        return None


@micropython.native
def main():
    STATUS_LED = PioBlinker(BLINKER_PIO, STATUS_LED_OUT)
    LED = RGB(ADDR_LED_OUT, 9, 50)
    LED.set_mode(RGB.MODE_STATIC)

    brain = VexBrain(RS485_UART, RS485_EN_OUT)

    # Wait till otos connected
    STATUS_LED.blink(2)
    LED.set_color(RGB.RED)
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

    LED.set_color(RGB.ORANGE)
    otos.calibrate()

    LED.set_color(RGB.GREEN)
    STATUS_LED.blink(1)  # Only blink once to show success
    i = 0
    while True:
        if TEST_MODE and i % 1000 == 0:
            pos = struct.unpack("<3h", otos.get_position())
            print(
                f"X: {pos[0] / 32767 * 10}m, Y: {pos[1] / 32767 * 10}m, H: {pos[2] / 32767 * 180}deg"
            )

        # TODO: UART RX_IDLE interrupt, preferably hardware for ultimate speedy communication
        data = brain.receive()
        if data is not None:
            request_id = data[0]
            if request_id == REQ_GET_POSITION:
                brain.send(otos.get_position())
            elif request_id == REQ_SET_POSITION:
                otos.set_position(data[1:])
                brain.send(b"d")
            elif request_id == REQ_GET_VELOCITY:
                brain.send(otos.get_velocity())
            elif request_id == REQ_CALIBRATE:
                otos.calibrate()
                brain.send(b"d")
            elif request_id == REQ_SET_OFFSETS:
                otos.set_offset(data[1:])
                brain.send(b"d")
            elif request_id == REQ_SET_SCALARS:
                otos.set_scalar(data[1:])
                brain.send(b"d")
            elif request_id == REQ_GET_STDDEV:
                brain.send(otos.get_stddev())
            elif request_id == REQ_PING:
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
            elif request_id == REQ_SET_LEDS:
                color = cast(int, struct.unpack("<I", data[1:])[0])
                if color == LEDS_RAINBOW_STATIC:
                    LED.set_mode(RGB.MODE_STATIC)
                    LED.set_rainbow()
                elif color == LEDS_RAINBOW_ROTATE:
                    LED.set_mode(RGB.MODE_ROTATE)
                    LED.set_rainbow()
                else:
                    LED.set_color(color)

        if i % 2500 == 0:
            LED.update()
        i += 1 # TODO stop using this and use an actual clock

        time.sleep(0)  # Yield to scheduler


if __name__ == "__main__":
    while True:
        try:
            main()
        except Exception as e:
            sys.print_exception(e)
            time.sleep(1)
