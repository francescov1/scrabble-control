from armSim import Arm, Joint
from motorControl import MCU, MSG
from math import pi

from keyboard import add_hotkey
import keyboard

try:
    from picamera import PiCamera as camera
    from RPi.GPIO import GPIO
except:
    print("Failed to import picam and/or GPIO. Will continue.")

from time import time, sleep

SERIAL_PORT = "/dev/ttyAMA0"

def buttonCallback(channel):
    pass

def setupGpio():
    GPIO.setmode(GPIO.BOARD)

def setupCamera():
    camera.resolution = (2592, 1944)
    camera.start_preview()
    # Camera warm-up time
    sleep(2)

def setupArm():
    arm = Arm()
    base = Joint('base', (0,0,2.28))
    base.dynamic('translation', axis='y')
    shoulder = Joint('shoulder', (0,0,10))
    shoulder.attach(base)
    shoulder.dynamic(type='rotation', axis='y', range=(-pi/2,pi/2))
    elbow = Joint('elbow', (0,0,12))
    elbow.attach(shoulder)
    elbow.dynamic(type='rotation', axis='y', range=(-pi,pi))
    wrist = Joint('wrist', (0,0,2))
    wrist.attach(elbow)
    wrist.dynamic('rule', axis='y', rule=(0,0,-1))
    #shoulder.rotate('y', 1.507)
    #elbow.rotate('y', -1.488)
    #wrist.check_rule()
    #print(wrist.R_angle()['y'])
    #wrist.rotate('y', 3.059)
    arm.add(base)
    arm.add(shoulder)
    arm.add(elbow)
    arm.add(wrist)
    return arm


def manual():
    arduino = MCU(port=SERIAL_PORT)
    def move(motor, dir):
        sp = arduino.get(motor, MSG.INFO.SETPOINT)
        print(sp)
        #sp += dir
        #print("Setting M{} to {}".format(motor, sp))
        #arduino.set(motor, sp)
    
    print("Ready for manual control")
    add_hotkey('b+up', move, args=[MSG.MOTOR.BASE, 1])
    add_hotkey('s+up', move, args=[MSG.MOTOR.SHOULDER, 1])
    add_hotkey('e+up', move, args=[MSG.MOTOR.ELBOW, 1])
    add_hotkey('w+up', move, args=[MSG.MOTOR.WRIST, 1])
    add_hotkey('b+down', move, args=[MSG.MOTOR.BASE, -1])
    add_hotkey('s+down', move, args=[MSG.MOTOR.SHOULDER, -1])
    add_hotkey('e+down', move, args=[MSG.MOTOR.ELBOW, -1])
    add_hotkey('w+down', move, args=[MSG.MOTOR.WRIST, -1])
    keyboard.wait('esc')

def main():
    arm = setupArm()

    bounding_box = ((0, float('Inf')), None, (0, float('Inf')))
    arm.generateDatabase(100, debug=False, memory=False, bounding_box=bounding_box)
    #arm.importDatabase('2019-10-09-191803.db')
    angles, delta = arm.motionControl((10,0,5))
    arm.plot()
    setupCamera()
    camera.capture(path)


if __name__ == '__main__':
    manual()
