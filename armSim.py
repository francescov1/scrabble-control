import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
from math import pi, sin, cos, sqrt

from objects import Joint

class Arm:
    def __init__(self):
        self.parts = {}

    def learn(self, dataset):
        
        Yx = [pt[0] for pt in dataset[:][-1]]
        Yy = [pt[1] for pt in dataset[:][-1]]
        Yz = [pt[2] for pt in dataset[:][-1]]
        print(X)

    def generateTraining(self, resolution=10):
    #only for parts that rotate
        rotating_parts = []
        for key, part in self.parts.items():
            if not part.dof.fixed and part.dof.type == 'rotation':
                rotating_parts.append(part)

        dataset_width = len(rotating_parts) + 3
        dataset = np.zeros(shape=(1,dataset_width))
        def add_data_point():
            nonlocal dataset
            dpt = np.zeros(shape=(1, dataset_width))
            for i in range(len(rotating_parts)):
                dpt[0][i] = rotating_parts[i].angle[rotating_parts[i].dof.axis]
            endpoint = rotating_parts[-1].mapFrom(rotating_parts[-1].r).reshape(1,-1)[0]
            dpt[0][-3:-1] = endpoint[-3:-1]
            dataset = np.append(dataset, dpt, axis=0)

        def generator(parts):
            index = 0
            for part in parts:
                range = np.linspace(part.dof.range[0], part.dof.range[1], resolution)
                for angle in range:
                    part.rotate(part.dof.axis, angle)
                    if len(parts)-index > 1:
                        generator(parts[1:])
                    add_data_point()
                    #reset
                    part.undo
                index+=1

        generator(rotating_parts)
        return dataset

    #non-general function based on specific arm shape
    def motionControl(self, coord):
        coord = np.array(coord)
        base = self.parts['base']
        shoulder = self.parts['shoulder']
        elbow = self.parts['elbow']
        wrist = self.parts['wrist']

        length = sqrt(np.sum(np.power(coord, 2)))

        if length > (shoulder.length()+elbow.length()):
            print('Coordinate specified is out of reach')
            return

        best_s_a = 0
        best_e_a = 0
        current_best = 10
        axis = 'y'
        for shoulder_angle in np.linspace(-pi/2, pi/2, num=20):
            for elbow_angle in np.linspace(-pi/2, pi/2, num=20):
                shoulder.rotate(axis, shoulder_angle)
                elbow.rotate(axis, elbow_angle)
                r = wrist.mapFrom(wrist.r)
                distance_from_target = sqrt(np.sum(np.power(np.subtract(coord, r),2)))
                #print(distance_from_target)
                if distance_from_target < current_best:
                    current_best = distance_from_target
                    best_e_a = elbow_angle
                    best_s_a = shoulder_angle
                #reset
                shoulder.rotate(axis, -shoulder_angle)
                elbow.rotate(axis, -elbow_angle)

        print('Optimal shoulder angle: {}'.format(best_s_a))
        print('Optimal elbow angle: {}'.format(best_e_a))
        print('Closest approach: {}'.format(current_best))
        shoulder.rotate(axis, best_s_a)
        elbow.rotate(axis, best_e_a)
        self.plot()

    def plot(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for key, obj in self.parts.items():
            origin = np.squeeze(obj.ref_frame.mapFrom(obj.origin))
            endpoint = np.squeeze(obj.mapFrom(obj.r))
            ax.plot(xs=(origin[0],endpoint[0]),
                    ys=(origin[1],endpoint[1]),
                    zs=(origin[2],endpoint[2]))

        lim = [-1.5,1.5]
        ax.set_xlim(lim)
        ax.set_ylim(lim)
        ax.set_zlim([0, max(lim)])

        ax.set_xlabel(r'$X$')
        ax.set_ylabel(r'$Y$')
        ax.set_zlabel(r'$Z$')

        plt.show()

    def add(self, object):
        if object.name in self.parts.keys():
            raise KeyError
        else:
            self.parts[object.name] = object


def armSetup():
    arm = Arm()
    base = Joint('base', (0,0,0))
    shoulder = Joint('shoulder', (0,0,1))
    shoulder.dynamic('rotation', 'x', (-pi/2,pi/2))
    shoulder.attach(base)
    elbow = Joint('elbow',(1,0,0))
    elbow.dynamic('rotation', 'x', (-pi/2,pi/2))
    elbow.attach(shoulder)
    wrist = Joint('wrist', (0.25,0,0))
    wrist.attach(elbow)

    arm.add(base)
    arm.add(shoulder)
    arm.add(elbow)
    arm.add(wrist)
    return arm

arm = armSetup()
training_data = arm.generateTraining(100)
print(training_data)
#arm.learn(training_data)
#arm.motionControl((1,1,0.5))

#arm.plot()
