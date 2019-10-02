import serial
from queue import Queue
from threading import Thread, RLock, Event
from time import sleep, time
import struct


class MCU:
    MSG_SIZE = 3
    class TX:
        HEARTBEAT = b'\x00\x00\x00'
        SET = b'\x01'
        GET = b'\x02'
    
    class RX:
        FAILED = b'\x00'
        SUCCESS = b'\x01'
        
    class MOTOR:
        BASE = b'\x00'
        SHOULDER = b'\x01'
        ELBOW = b'\x02'
        WRIST = b'\x03'
        SUCTION = b'\x04'
        
    def __init__(self, port):
        self.port = port
        self.Serial = serial.Serial()
        self.lock = RLock()
        self.__open_serial()
        self.active = Event()
        
        # Start
        self.heartbeat()
    
    def heartbeat(self):
        def run(self):
            i = 0
            while i < 5:
                i = i+1
                resp = self.send(MCU.TX.HEARTBEAT)
                if resp == MCU.TX.HEARTBEAT:
                    self.active.set()
                    i = 0
                else:
                    self.active.clear()
                sleep(2)
            print('5 heartbeats missed: resetting')
                
        t = Thread(target=run, args=(self,))
        t.start()
    
    def __open_serial(self, baudrate=115200, timeout=0, write_timeout=0):
        self.lock.acquire()
        if not self.Serial.is_open:
            self.Serial.port = self.port
            self.Serial.baudrate = baudrate
            self.Serial.timeout = timeout
            self.Serial.write_timeout = write_timeout
            self.Serial.open()
        self.lock.release()
    
    def __close_serial(self):
        self.lock.acquire()
        if self.Serial.is_open:
            self.Serial.close()
        self.lock.release()
    
    def reset(self):
        self.__close_serial()
        self.__open_serial()
    
    def send(self, msg, timeout=1):
        with self.lock:
            constructor = bytearray(MCU.MSG_SIZE)
            constructor[:len(msg)] = msg
            self.Serial.write(constructor)
            start_time = time()
            while self.Serial.in_waiting == 0:
                if time()-start_time > timeout:
                    return None
                sleep(0.1)
            resp = self.Serial.read(size=self.Serial.in_waiting)
            return resp
    
    def set_motor(self, motor, value):
        msg = MCU.TX.SET + motor + value.to_bytes(1, byteorder='little')
        resp = mcu.send(msg)
        if resp == MCU.RX.SUCCESS:
            print('Motor set to {}'.format(value))
        else:
            print('Error setting motor', resp)
    
mcu = MCU(port='COM4')
mcu.set_motor(MCU.MOTOR.BASE, 1)
