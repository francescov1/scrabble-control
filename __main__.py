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

#SERIAL_PORT = "/dev/ttyAMA0"
SERIAL_PORT = "COM7"

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
    base = Joint('base', (0,0,62))
    base.dynamic('translation', axis='y')
    shoulder = Joint('shoulder', (231,0,0))
    shoulder.attach(base)
    shoulder.dynamic(type='rotation', axis='y', range=(0,pi/2))
    elbow = Joint('elbow', (207,0,0))
    elbow.attach(shoulder)
    elbow.dynamic(type='rotation', axis='y', range=(-pi,0))
    wrist = Joint('wrist', (82,0,0))
    wrist.attach(elbow)
    wrist.dynamic('rule', axis='y', rule=(0,0,-1))
    arm.add(base)
    arm.add(shoulder)
    arm.add(elbow)
    arm.add(wrist)
    return arm


def manual():
    arduino = MCU(port=SERIAL_PORT)
    def move(motor, dir):

        sp = arduino.get(motor, MSG.INFO.SETPOINT).dataAsInt()
        sp += dir * 2
        if sp < 0:
            sp == 0
        print("Setting M{} to {}".format(motor, sp))
        arduino.set(motor, sp)

    def suction():
        arduino.set(MSG.MOTOR.SUCTION, 0)

    print("Ready for manual control")
    add_hotkey('right', move, args=[MSG.MOTOR.BASE, 1])
    add_hotkey('s+up', move, args=[MSG.MOTOR.SHOULDER, 1])
    add_hotkey('e+up', move, args=[MSG.MOTOR.ELBOW, 1])
    add_hotkey('w+up', move, args=[MSG.MOTOR.WRIST, 1])
    add_hotkey('left', move, args=[MSG.MOTOR.BASE, -1])
    add_hotkey('s+down', move, args=[MSG.MOTOR.SHOULDER, -1])
    add_hotkey('e+down', move, args=[MSG.MOTOR.ELBOW, -1])
    add_hotkey('w+down', move, args=[MSG.MOTOR.WRIST, -1])
    add_hotkey('space', suction)
    keyboard.wait('esc')


def commandMotors(arduino, arm, endpoint):
    x = endpoint[0]
    y = endpoint[1]
    z = endpoint[2]

    target = (y, 0, z)
    angles, delta = arm.motionControl(target)

    arduino.set(MSG.MOTOR.BASE, x)
    motorMap = [MSG.MOTOR.SHOULDER, MSG.MOTOR.ELBOW, MSG.MOTOR.WRIST]
    offset = [0, pi, -pi/2]
    for i in range(len(motorMap)):
        angle = int((angles[i] + offset[i]) * 180/pi)
        if angle < 0:
            angle = 0
        print(angle)
        arduino.set(motorMap[i], angle)
        #print(arduino.get(motorMap[i], MSG.SETPOINT))


def main():
    arduino = MCU(port=SERIAL_PORT)
    arm = setupArm()

if __name__ == '__main__':
    bounding_box = ((0, float('Inf')), (0, float('Inf')), (0, float('Inf')))
    arm.generateDatabase(50, debug=False, memory=True, bounding_box=bounding_box)
    #arm.importDatabase('main.db')
    manual()
