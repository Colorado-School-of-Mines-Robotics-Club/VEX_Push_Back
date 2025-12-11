import machine
# import qwiic_otos
from otos import OtosSensor
import sys
import time
from neopixel import NeoPixel
import cobs

RS485_UART = machine.UART(2, baudrate=115200, tx=17, rx=16)
RS485_EN_PIN = machine.Pin(23, machine.Pin.OUT)
LED_PIN = machine.Pin(14, machine.Pin.OUT)
OTOS_I2C = machine.I2C(sda=machine.Pin(21), scl=machine.Pin(22), freq=100000)

LED_BLACK = 0
LED_RED = 1
LED_BLUE = 2
LED_RAINBOW = 3
LED_ROTATE = 4
LED_GREEN = 5

class RGB():
    def __init__(self, pin, length, brightness):
        self.np = NeoPixel(pin, length)
        self.brightness = brightness
        self.set_mode(LED_BLACK)

    def set_mode(self, mode: int):
        self.mode = mode
        self.update()

    def update(self):
        if self.mode == LED_BLACK:
            for i in range(len(self.np)):
                self.np[i] = (0,0,0)
        elif self.mode == LED_RED:
            for i in range(len(self.np)):
                self.np[i] = (int(255/self.brightness),0,0)
        elif self.mode == LED_BLUE:
            for i in range(len(self.np)):
                self.np[i] = (0,int(255/self.brightness),0)
        elif self.mode == LED_RAINBOW:
            for i in range(len(self.np)):
                pos = int((i * 256) / len(self.np)) % 256
                if pos < 85:
                    c= (int((pos * 3)/self.brightness), int((255 - pos * 3)/self.brightness), 0)
                elif pos < 170:
                    pos -= 85
                    c= (int((255 - pos * 3)/self.brightness), 0, int((pos * 3)/self.brightness))
                else:
                    pos -= 170
                    c= (0, int((pos * 3)/self.brightness), int((255 - pos * 3)/self.brightness))
                self.np[i]=c
            self.mode=4
        elif self.mode == LED_ROTATE:
            last=self.np[0]
            for i in range(len(self.np)-1):
                self.np[i] = self.np[i+1]
            self.np[-1]=last
        elif self.mode == LED_GREEN:
            for i in range(len(self.np)):
                self.np[i] = (0,0,int(255/self.brightness))

        self.np.write()

class VexBrain():
    def __init__(self, uart, enable_pin):
        self.uart = uart
        self.enable_pin = enable_pin
        self.buffer = bytearray()

    def send(self, data: bytes):
        self.enable_pin.value(1)
        self.uart.write(cobs.encode(data))
        self.uart.flush()
        self.enable_pin.value(0)

    def receive(self):
        for i in range(self.uart.any()):
            self.buffer.append(self.uart.read(1)[0])
            if self.buffer[-1] == 0x00:
                encoded = bytes(self.buffer)
                self.buffer = bytearray()
                return cobs.decode(encoded)
        return None

def main():
    LED = RGB(LED_PIN, 32, 1)
    LED.set_mode(LED_BLACK)
    time.sleep(.5)
    brain = VexBrain(RS485_UART, RS485_EN_PIN)
    otos = OtosSensor(OTOS_I2C)
    otos.calibrate()

    led_mode = 0
    i = 0
    while True:
        data = brain.receive()
        if data != None:
            print(data)
            if data[0:1] == b'p':
                brain.send(otos.get_position())
            elif data[0:1] == b'v':
                brain.send(otos.get_velocity())
            elif data[0:1] == b'c':
                otos.calibrate()
                brain.send(b'd')
            elif data[0:1] == b'o':
                otos.set_offset(data[1:])
                brain.send(b'd')
            elif data[0:1] == b's':
                otos.set_scalar(data[1:])
                brain.send(b'd')
            elif data[0:1] == b'a':
                brain.send(b'd')
            elif data[0:1] == b'l':
                led_mode = (led_mode + 1) % 6
                LED.set_mode(led_mode)
                brain.send(b'd')
        if i % 10 == 0:
            LED.update()
        i += 1

        time.sleep(0.001)

if __name__ == '__main__':
    while True:
        try:
            main()
        except Exception as e:
            sys.print_exception(e)
            time.sleep(1)
