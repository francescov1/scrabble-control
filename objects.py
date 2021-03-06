import numpy as np
import math
from math import pi, sin, cos, sqrt, acos
from collections import namedtuple
from copy import copy

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
        self.angle = {'x':0,'y':0,'z':0}

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
        self.angle = {'x':0,'y':0,'z':0}
        if self.ref_frame == None:
            self.ref_frame = GlobalRefFrame()
            self.compute_transform()
        else:
            self.change_ref_frame(ref_frame)

        self.intial_state = namedtuple('PrevState', ['u_v', 'transform', 'origin', 'angle'])
        self.backup()

    def backup(self):
        self.intial_state.u_v = copy(self.u_v)
        self.intial_state.origin = copy(self.origin)
        self.intial_state.transform = copy(self.transform)
        self.intial_state.angle = copy(self.angle)

    def reset(self):
        self.u_v = copy(self.intial_state.u_v)
        self.transform = copy(self.intial_state.transform)
        self.origin = copy(self.intial_state.origin)
        self.angle = copy(self.intial_state.angle)

    def magnitude(self, vector):
        return sqrt(np.sum(np.power(vector, 2)))

    def change_ref_frame(self, ref_frame):
        self.ref_frame = ref_frame
        self.compute_transform()

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
        self.angle[axis] += angle
        self.u_v = np.dot(R, self.u_v)
        self.compute_transform()

    def translate(self, origin):
        self.origin = np.asarray(origin).reshape(3,1)


class Joint(RefFrame):
    def __init__(self, name, r):
        self.r = np.array(r).reshape(3,1)
        #DOF=Degree of freedom. Used when calculating motion automatically
        self.dof = namedtuple('DoF', ['type', 'axis', 'range', 'r', 'fixed'])
        self.dof.fixed = True
        RefFrame.__init__(self, name)

    def backup(self):
        self.backup_r = copy(self.r)
        np.testing.assert_array_equal(self.r, self.backup_r)
        RefFrame.backup(self)

    def reset_r(self):
        self.r = copy(self.backup_r)
        np.testing.assert_array_equal(self.r, self.backup_r)

    def reset(self):
        self.reset_r()
        RefFrame.reset(self)

    def __str__(self):
        return ('Joint.{}'.format(self.name))
    def __repr__(self):
        return self.__str__()

    def dynamic(self, type, axis, range=(0,0), rule=(0,0,0)):
        axis = axis.lower()
        type = type.lower()
        assert axis in ['x','y','z']
        #Rotation: rotate about axis from range[0] to range[1]
        #Tranlation: move origin on axis from range[0] to range[1]
        #Rule: Fix vector endpoint (self.r)
        assert type in ['rotation','translation', 'rule']
        self.dof.type = type
        self.dof.axis = axis
        self.dof.range = range
        self.dof.rule = np.array(rule).reshape(3,1)
        self.dof.fixed = False

    def attach(self, obj):
        self.change_ref_frame(obj)
        self.translate(obj.r)
        self.backup() #new intial state

    def rotate(self, axis, angle):
        RefFrame.rotate(self, axis, angle)
        #self.update_angles()

    def translate(self, origin):
        RefFrame.translate(self, origin)
        #self.update_angles()

    def check_rule(self):
        self.reset_r()
        if self.dof.type != 'rule':
            print('No rule defined')
            return
        act_origin = self.ref_frame.mapFrom(self.origin)
        target = self.mapTo(np.add(self.dof.rule,act_origin))
        target = target/self.magnitude(target)
        assert round(self.magnitude(target), 10) == 1, self.magnitude(target)
        self.r = np.multiply(target, self.magnitude(self.r))

    def R_angle(self):
        #works for single axis rotation, wouldn't trust it for multi-axis
        #kind of hacky, only to be used after solving rule for r
        Rxy = np.multiply(self.r, np.matrix("1;1;0"))
        Rxz = np.multiply(self.r, np.matrix("1;0;1"))
        #print(Rxz)
        Ryz = np.multiply(self.r, np.matrix("0;1;1"))
        def calc_angle(a, b):
            a = a.transpose()
            a_mag = self.magnitude(a)
            b_mag = self.magnitude(b)
            if a_mag * b_mag == 0:
                return 0
            dot = np.dot(a,b)
            # avoid multipying by zero to preserve sign
            #a[a==0]=1
            sign = 1# if np.prod(a) > 0 else -1
            return sign * acos(dot/(a_mag*b_mag))
        angles = {}
        #angles['x'] = calc_angle(Ryz, np.matrix("0;0;1"))
        angles['y'] = calc_angle(Rxz, np.matrix("0;0;1"))
        #angles['z'] = calc_angle(Rxy, np.matrix("1;0;0"))
        return angles

    def length(self):
        act_origin = self.ref_frame.mapFrom(self.origin)
        a = np.subtract(self.mapFrom(self.r), act_origin)
        return sqrt(np.sum(np.power(a, 2)))
