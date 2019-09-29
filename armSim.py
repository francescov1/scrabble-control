import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
from math import pi, sin, cos, sqrt
from collections import namedtuple
import copy

def build_u_v(u_v):
    return (np.array(u_v[0]).reshape(3,1),
            np.array(u_v[1]).reshape(3,1),
            np.array(u_v[2]).reshape(3,1))

class GlobalRefFrame:
    def __init__(self):
        self.name = 'GFR'
        self.origin = np.array((0,0,0)).reshape(3,1)
        self.u_v = build_u_v(((1,0,0),(0,1,0),(0,0,1)))
        self.transform = np.identity(3)
    
    def mapTo(self, coord):
        return coord

    def mapFrom(self, coord):
        return coord

class RefFrame:
    def __init__(self, name, origin=(0,0,0), u_v=((1,0,0),(0,1,0),(0,0,1)), ref_frame=None):
        self.name = name
        self.u_v = build_u_v(u_v)
        self.origin = np.array(origin).reshape(3,1)
        self.ref_frame = ref_frame
        if self.ref_frame == None:
            self.ref_frame = GlobalRefFrame()
            self.compute_transform()
        else:
            self.change_ref_frame(ref_frame)
        
        self.prev_state = namedtuple('PrevState', ['u_v', 'transform', 'origin'])
        self.__backup()
        
    def __backup(self):
        self.prev_state.u_v = self.u_v
        self.prev_state.origin = self.origin
        self.prev_state.transform = self.transform
    
    def undo(self):
        self.u_v = self.prev_state.u_v
        self.transform = self.prev_state.transform
        self.origin = self.prev_state.origin
    
    def magnitude(self, vector):
        return sqrt(np.sum(np.power(vector, 2)))
    
    def change_ref_frame(self, ref_frame):
        self.ref_frame = ref_frame
        self.compute_transform()
        self.__backup() #don't allow an undo from this state
    
    def compute_transform(self):
        global_u_v = np.array(((1,0,0),(0,1,0),(0,0,1)))
        self.transform = np.zeros((3,3))
        for i in range(3):
            for j in range(3):
                u_v = np.squeeze(self.u_v[j].reshape(1,3))
                self.transform[i][j] = np.dot(global_u_v[i], u_v)
    
    def test(self, iterations = 10):
        for u_v in self.u_v:
            assert round(self.magnitude(u_v), 6) == 1, "{}".format(self.magnitude(u_v))
        
        for i in range(iterations):
            coord = np.random.randint(100, size=(3,1))
            reverse_coord = self.mapFrom(self.mapTo(coord))
            np.testing.assert_array_almost_equal(coord, reverse_coord, decimal=6)
            assert (coord.shape == (3,1))
            
    def __listRefFrames(self):
        rfList = list()
        ref_frame = self
        while True:
            rfList.append(ref_frame)
            # need to find better equality
            if type(ref_frame) == GlobalRefFrame:
                break
            ref_frame = ref_frame.ref_frame
        return rfList
        
    def mapFrom(self, coord):
        coord = np.asarray(coord)
        if coord.shape != (3,1):
            coord = coord.reshape(3,1)
        for ref_frame in self.__listRefFrames():
            #print('****REF FRAME****',ref_frame.name)
            #print('coord before\n',coord)
            #print('origin\n',ref_frame.origin)
            #print('transform\n',ref_frame.transform)
            coord = np.add(np.dot(np.linalg.inv(ref_frame.transform), coord), ref_frame.origin)
            #print('coord after\n',coord)
        return coord
    
    def mapTo(self, coord):
        coord = np.asarray(coord)
        if coord.shape != (3,1):
            coord = coord.reshape(3,1)
        for ref_frame in self.__listRefFrames()[::-1]:
            #print('****REF FRAME****',ref_frame.name)
            #print('coord before\n',coord)
            #print('origin\n',ref_frame.origin)
            #print('transform\n',ref_frame.transform)
            coord = np.dot(ref_frame.transform, np.subtract(coord,ref_frame.origin))
            #print('coord after\n',coord)
        return coord
        
    def rotate(self, axis, angle):
        sin_a = sin(angle)
        cos_a = cos(angle)
        
        if axis.lower()=='x':
            R = np.array([[1, 0, 0],[0, cos_a, -sin_a],[0, sin_a, cos_a]])
        elif axis.lower()=='y':
            R = np.array([[cos_a, 0, sin_a],[0, 1, 0],[-sin_a, 0, cos_a]])
        elif axis.lower()=='z':
            R = np.array([[cos_a, -sin_a, 0],[sin_a, cos_a, 0],[0, 0, 1]])
        else:
            print("{} not recognized".format(axis))
            return
        self.__backup()
        self.u_v = np.dot(R, self.u_v)
        self.compute_transform()
        
    def translate(self, origin):
        self.__backup()
        self.origin = origin

# GFR = RefFrame()
# f1 = RefFrame(origin=(1,0,1))
# f1.change_ref_frame(GFR)
# f2 = RefFrame()
# f2.change_ref_frame(f1)
# f1.rotate('x', pi/4)
# f2.rotate('x', pi/4)

# t2 = f2.mapTo((0,1,1))
# t3 = f2.mapFrom(t2)
# print('mapTo\n',t2)
# print('mapFrom\n',t3)

# exit()

class Arm:
    def __init__(self):
        self.parts = {}
    
    def solver(self, coord):
        best_s_a = 0
        best_e_a = 0
        current_best = 1e10
        for key, part in self.parts:
            if part.dof.fixed:
                break
            if part.dof.type == 'rotation':
                range = p.linspace(part.dof.range[0],part.dof.range[1])
                for angle in range:
                    part.rotate(part.dof.axis, angle)
                    
        return best_s_a, best_e_a
    
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
    
class Joint(RefFrame):
    def __init__(self, name, r):
        RefFrame.__init__(self, name)
        self.r = np.array(r).reshape(3,1)
        self.dof = namedtuple('DoF', ['type', 'axis', 'range', 'fixed'])
        self.dof.fixed = True
    
    def dynamic(self, type, axis, range):
        axis = axis.lower()
        type = type.lower()
        assert axis in ['x','y','z']
        assert type in ['rotation, translation']
        self.dof.type = type
        self.dof.axis = axis
        self.dof.range = range
        self.dof.fixed = False
    
    def attach(self, obj):
        self.change_ref_frame(obj)
        self.translate(obj.r)
    
    def rotate(self, axis, angle):
        RefFrame.rotate(self, axis, angle)
    
    def translate(self, origin):
        RefFrame.translate(self, origin)
        
    def length(self):
        a = np.subtract(self.mapFrom(self.r), self.origin)
        return sqrt(np.sum(np.power(a, 2)))
    
    

def armSetup():
    arm = Arm()
    base = Joint('base', (0,0,0))
    shoulder = Joint('shoulder', (0,0,1))
    shoulder.attach(base)
    elbow = Joint('elbow',(1,0,0))
    elbow.attach(shoulder)
    wrist = Joint('wrist', (0.25,0,0))
    wrist.attach(elbow)

    arm.add(base)
    arm.add(shoulder)
    arm.add(elbow)
    arm.add(wrist)
    return arm
    
arm = armSetup()
#arm.motionControl((1,1,0.5))

arm.plot()