import machine
import qwiic_otos
import sys
import time
from neopixel import NeoPixel   
#import cobs
a=0
uart = machine.UART(2, baudrate=115200, tx=17, rx=16)
rs485_dir = machine.Pin(23, machine.Pin.OUT)
LED_pin = machine.Pin(14, machine.Pin.OUT) 
sdaPIN = machine.Pin(21)
sclPIN = machine.Pin(22)
i2c = machine.I2C(sda=sdaPIN, scl=sclPIN, freq=100000)

class RGB():
    def __init__(self, pin, ammout,bright):
        self.np = NeoPixel(pin, ammout)
        self.brightness = bright
        self.mode = 0
    def set_mode(self,mode):
        self.mode = mode
        self.update()
    def update(self):
        if self.mode == 0:
            for i in range(len(self.np)):
                self.np[i] = (0,0,0)
        elif self.mode == 1:
            for i in range(len(self.np)):
                self.np[i] = (int(255/self.brightness),0,0)
        elif self.mode == 2:
            for i in range(len(self.np)):
                self.np[i] = (0,int(255/self.brightness),0)
        elif self.mode == 3:
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
        elif self.mode == 4:
            last=self.np[0]
            for i in range(len(self.np)-1):
                self.np[i] = self.np[i+1]
            self.np[-1]=last
        elif self.mode == 5:
            for i in range(len(self.np)):
                self.np[i] = (0,0,int(255/self.brightness))
        self.np.write()

class brain():
    def __init__(self,urat,on_off):
        self.urat = urat
        self.on_off = on_off
    def send(self,data):
        self.on_off.value(1)
        self.urat.write(data)
        time.sleep(.001)
        self.on_off.value(0)
    def receive(self):
        if self.urat.any():
            data = self.urat.read()
            return data
        else:
            return None

class fancysenser():
    def __init__(self,senser,output,led):
        self.senser = senser
        self.output = output
        self.senser.begin()
        if self.senser.is_connected() == False:
            print("The device isn't connected to the system. Please check your connection")
            led.set_mode(1)
            sys.exit()
        else:
            print("Device connected")
            led.set_mode(2)
        time.sleep(.1)
    def calabrate(self):
        print("Calibrating")
        self.senser.calibrateImu()
        time.sleep(.25)
        LED.set_mode(5)
    def reqest(self,type,extra):
        if type == 1:
            calabrate()
            self.output.send(b'd')
        elif type == 2:
            pos = self.senser.getPosition()
            self.output.send(b'%f%f%f' % (pos.x, pos.y, pos.h))
        elif type == 3:
            self.output.send(b'%f' % self.senser.getvelocity())
        elif type == 4:
            self.output.send(b'd')
        elif type == 5:
            self.senser.setoffset(extra)
            self.output.send(b'd')
        elif type == 6:
            self.senser.resetTracking()
            self.output.send(b'd')
        elif type == 7:
            self.senser.setscale(extra)
            self.output.send(b'd')
        
if __name__ == '__main__':
    LED=RGB(LED_pin,16,1)
    LED.set_mode(0)
    time.sleep(.5)
    brain=brain(uart,rs485_dir)
    otos=fancysenser(qwiic_otos.QwiicOTOS(),brain,LED)
    otos.calabrate()
    while True:
        data=brain.receive()
        if data != None:
            print(data)
            if data[0:1]==b'p':
                otos.reqest(2,0)
            elif data[0:1]==b'v':
                otos.reqest(3,0)
            elif data[0:1]==b'c':
                otos.reqest(1,0)
            elif data[0:1]==b'o':
                extra=float(data[1:].decode('utf-8'))
                otos.reqest(5,extra)
            elif data[0:1]==b's':
                extra=float(data[1:].decode('utf-8'))
                otos.reqest(7,extra)
            elif data[0:1]==b't':
                otos.reqest(6,0)
            elif data[0:1]==b'a':
                otos.reqest(4,0)
            elif data[0:1]==b'l':
                a+=1
                if a%4==0:
                    LED.set_mode(0)
                elif a%4==1:
                    LED.set_mode(1)
                elif a%4==2:
                    LED.set_mode(2)
                else:
                    LED.set_mode(3)
        LED.update()
        