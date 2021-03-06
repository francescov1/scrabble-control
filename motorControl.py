import serial
from queue import Queue
from threading import Thread, RLock, Event
from time import sleep, time
from struct import pack, unpack

class MSG(bytearray):
    SIZE = 6  #*8 bit
    def __init__(self, source=None, msgType=None, id=None, data=None):
        if source == None:
            source = MSG.SIZE
        super().__init__(source)
        if msgType != None:
            self[0] = int.from_bytes(msgType, 'big')
        if id != None:
            self[1] = int.from_bytes(id, 'big')
        if data != None:
            self.dataEncode(data);
    def type(self):
        return int(self[0]).to_bytes(1, 'big')
    def id(self):
        return int(self[1]).to_bytes(1, 'big')
    def data(self):
        return self[2:]
    def dataAsInt(self):
        return unpack('<l', self[2:])[0]
        #return int.from_bytes(self.data(), 'big')
    def dataEncode(self, value):
        if type(value) == bytes:
            self[-1] = int.from_bytes(value, 'big')
        elif type(value) == int:
            self[2:] = pack('>l', value)
        else:
            print(type(value))
            raise Exception('TypeError')

    class TYPE:
        HEARTBEAT = b'\x00'
        SET = b'\x01'
        GET = b'\x02'
    class INFO:
        STATUS = b'\x00'
        ERROR = b'\x01'
        SUCCESS = b'\x02'
        ACK = b'\x03'
        SETPOINT = b'\x04'
        POSITION = b'\x05'

    class MOTOR:
        BASE = b'\x00'
        SHOULDER = b'\x01'
        ELBOW = b'\x02'
        WRIST = b'\x03'
        SUCTION = b'\x04'

    nonLatchedStatusFlag = {
        0: 'NO FAULT',
        1: 'POWER',
        (1 << 2):'OPENY',
        (1 << 3):'OPENX',
        (1 << 4):'WD',
        (1 << 5):'CPFAIL',
        (1 << 6):'TW'
    }

    latchedStatusFlag = {
        0: 'NO FAULT',
        1: 'POWER',
        (1 << 3):'OVCXNB',
        (1 << 4):'OVCXNT',
        (1 << 5):'OVCXPB',
        (1 << 6):'OVCXPT',
        (1 << 10):'TSD',
        (1 << 11):'OVCYNB',
        (1 << 12):'OVCYNT',
        (1 << 13):'OVCYPB',
        (1 << 14):'OVCYPT'
       }

class MCU:
    def __init__(self, port):
        self.port = port
        self.Serial = serial.Serial()
        self.lock = RLock()
        self.__open_serial()
        self.active = Event()
        self.error = Event()

        # Start
        #self.heartbeat()

    def heartbeat(self):
        def run(self):
            i = 0
            while i < 5:
                i = i+1
                msg = MSG(msgType=MSG.TYPE.HEARTBEAT)
                resp = self.__send(msg)
                if resp != None and  resp.type() == MSG.TYPE.HEARTBEAT:
                    self.active.set()
                    i = 0
                else:
                    self.active.clear()
                if resp.id != MSG.INFO.SUCCESS:
                    self.error.set()
                else:
                    self.error.clear()
                sleep(5)
            print('5 heartbeats missed: resetting')
            self.reset()
            self.heartbeat()

        t = Thread(target=run, args=(self,))
        t.daemon = True
        t.start()

    def __open_serial(self, baudrate=115200, timeout=0, write_timeout=0):
        self.lock.acquire()
        if not self.Serial.is_open:
            self.Serial.port = self.port
            self.Serial.baudrate = baudrate
            self.Serial.timeout = timeout
            self.Serial.write_timeout = write_timeout
            self.Serial.open()
            sleep(2)
        self.lock.release()

    def __close_serial(self):
        self.lock.acquire()
        if self.Serial.is_open:
            self.Serial.close()
        self.lock.release()

    def reset(self):
        self.__close_serial()
        sleep(0.1)
        self.__open_serial()

    def __send(self, msg, timeout=2):
        assert type(msg) == MSG
        with self.lock:
            #print("Sending: {}".format(msg))
            self.Serial.write(msg)
            start_time = time()
            while self.Serial.in_waiting < MSG.SIZE:
                if time()-start_time > timeout:
                    print("Response timed out")
                    print("Buffer: {}".format(self.Serial.read(size=self.Serial.in_waiting)))
                    return None #timeout condition
                sleep(0.1)
            resp = self.Serial.read(size=self.Serial.in_waiting)
            resp = MSG(resp)
            print("Received: {}".format(resp))
            return resp

    def set(self, id, value):
        msg = MSG(msgType=MSG.TYPE.SET, id=id, data=value)
        #print(msg)
        resp = self.__send(msg)
        assert resp.type() == MSG.TYPE.SET
        return resp

    def get(self, id, type):
        msg = MSG(msgType=MSG.TYPE.GET, id=id, data=type)
        #print(msg)
        resp = self.__send(msg)
        assert resp.type() == MSG.TYPE.GET
        return resp

    def status(self, id):
        msg = MSG(msgType=MSG.TYPE.GET, id=id, data=MSG.INFO.STATUS)
        resp = self.__send(msg)
        if resp != None:
            latched = int.from_bytes(resp.data()[0:2], 'big')
            nonLatched = int.from_bytes(resp.data()[2:4], 'big')
            latched = MSG.latchedStatusFlag[latched]
            nonLatched = MSG.nonLatchedStatusFlag[nonLatched]
            errorFlag = resp.type()
        return errorFlag, latched, nonLatched

if __name__ == '__main__':
    arduino = MCU("/dev/ttyAMA0")
    arduino.set(MSG.MOTOR.SHOULDER, 90)
    exit()
