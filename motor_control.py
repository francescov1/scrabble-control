import serial
from queue import Queue
from threading import Thread, Lock, Event
from time import sleep

class MCU:
    def __init__(self, port):
        self.port = port
        self.Serial = serial.Serial()
        self.lock = Lock()
        self.connected = Event()
        self.__open_serial()
        sleep(2)
        #self.heartbeat()
    
    def heartbeat(self):
        def run(self):
            while True:
                for i in range(5):
                    msg = 200
                    resp = self.send(msg)
                    print(resp)
                    sleep(2)
                    if resp == msg:
                        i = 0
                print('5 heartbeats missed')
                break
                
        t = Thread(target=run, args=(self,))
        t.start()
        t.join()
    
    def __open_serial(self, baudrate=115200, timeout=0, write_timeout=0):
        self.lock.acquire()
        if not self.Serial.is_open:
            self.Serial.port = self.port
            self.Serial.baudrate = baudrate
            self.Serial.timeout = timeout
            self.Serial.write_timeout = write_timeout
            self.Serial.open()
        self.connected.set()
        self.lock.release()
    
    def __close_serial(self):
        self.lock.acquire()
        if self.Serial.is_open:
            self.Serial.close()
        self.lock.release()
    
    def reset(self):
        self.__close_serial()
        self.__open_serial()
    
    def send(self, msg, timeout=10):
        self.lock.acquire()
        self.Serial.reset_input_buffer()
        self.Serial.reset_output_buffer()
        self.Serial.write(msg)
        sleep(1)
        if self.Serial.in_waiting > 0:
            resp = bytes(bytes(self.Serial.read(size=1)))
        else:
            resp = None
        self.lock.release()
        return resp
    
mcu = MCU(port='COM4')
while True:
    print(mcu.send(bytes(100)))
    sleep(2)