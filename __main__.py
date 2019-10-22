from armSim import Arm, Joint
from motorControl import MCU
from math import pi

import numpy as np
from time import time

SERIAL_PORT = "COM4"

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
    wrist.rotate('y', 3.059)
    arm.add(base)
    arm.add(shoulder)
    arm.add(elbow)
    arm.add(wrist)
    return arm

arm = setupArm()
#arduino = MCU(port=SERIAL_PORT)

bounding_box = ((0,float('Inf')), None, (0, float('Inf')))
arm.generateDatabase(100, debug=False, memory=False, bounding_box=bounding_box)
#arm.importDatabase('2019-10-09-191803.db')
print(arm.motionControl((10,0,5)))
arm.plot()
exit()
for x in np.linspace(4, 30, 10):
    for z in np.linspace(0, 6, 10):
        a = arm.motionControl((x,0,z))
        if a==None:
            print('x:',x,'z:',z)
            continue
        elif a[-1] > 0.75:
            print('x:',x,'z:',z, a)
            arm.plot()
