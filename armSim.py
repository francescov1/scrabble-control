import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
from math import pi, sin, cos, sqrt
import sqlite3
from datetime import datetime
import os

from objects import Joint

class Database:
    TBL_NAME = 'database'
    def __init__(self, path):
        self.open(path)

    def create(self, names):
        seperator = ' real,'
        names = seperator.join(names)+' real'
        self.c.execute('''CREATE TABLE {}
            ({}, X real, Y real, Z real)'''.format(Database.TBL_NAME, names))

    def keys(self):
        keys = self.c.execute("PRAGMA table_info({})".format(Database.TBL_NAME))
        keys = [a[1] for a in keys.fetchall()]
        return keys

    def insert(self, values):
        keys = self.keys()
        insertion_list = [None] * len(keys)
        for key, value in values.items():
            insertion_list[keys.index(key)] = value
        insertion_list = tuple(insertion_list)
        assert None not in insertion_list
        template = ','.join(['?']*len(keys))
        self.c.execute('INSERT INTO {} VALUES ({})'.format(Database.TBL_NAME, template), insertion_list)
        self.conn.commit()

    def find_match(self, target):
        col = self.keys()
        col = ','.join(col)
        filter = '('
        i = 0
        for axis in ['X','Y','Z']:
            filter += '({} BETWEEN {} AND {})'.format(axis, math.floor(target[i]), math.ceil(target[i]))
            i += 1
            if i < 3:
                filter += ' AND '
        filter += ')'
        query = 'select {} from {} where {}'.format(col, Database.TBL_NAME, filter)
        result = self.c.execute(query)
        best_delta = 100
        best_row = None
        for row in result.fetchall():
            delta = sqrt((row[-3]-target[0])**2+(row[-2]-target[1])**2+(row[-1]-target[2])**2)
            if delta < best_delta:
                best_delta = delta
                best_row = row
        return best_row, best_delta

    def open(self, path):
        self.conn = sqlite3.connect(path)
        self.c = self.conn.cursor()

    def close(self):
        self.conn.commit()
        self.conn.close()

    def __del__(self):
        self.close()

class Arm:
    def __init__(self):
        self.parts = {}
        self.dynamic_parts = []
        self.db = None

    def importDatabase(self, file):
        self.db = Database(file)
        assert not bool(set([part.name for part in self.dynamic_parts]).intersection(set(self.db.keys())))

    def generateDatabase(self, resolution=2, debug=False, memory=False):
        if memory:
            db_name = ':memory:'
        else:
            now = datetime.now()
            db_name = now.strftime("%Y-%m-%d-%H%M%S")+".db"
        self.db = Database(db_name)
    #only for parts that rotate
        for key, part in self.parts.items():
            if not part.dof.fixed and part.dof.type in ['rotation', 'rule']:
                self.dynamic_parts.append(part)
        self.db.create([part.name for part in self.dynamic_parts])

        def add_data_point():
            values = {}
            for part in self.dynamic_parts:
                if part.dof.type == 'rule':
                    values[part.name] = part.R_angle()[part.dof.axis]
                else:
                    values[part.name] = part.angle[part.dof.axis]
            endpoint = np.squeeze(part.mapFrom(part.r))
            values['X'] = endpoint[0]
            values['Y'] = endpoint[1]
            values['Z'] = endpoint[2]
            if debug:
                    self.plot()
                    print(values)
            self.db.insert(values)

        def generator(parts):
            index=0
            for part in parts:
                if part.dof.type == 'rotation':
                    range = np.linspace(part.dof.range[0], part.dof.range[1], resolution)
                    for angle in range:
                        part.rotate(part.dof.axis, angle)
                        assert part.angle[part.dof.axis] == angle
                        #print('rotated')
                        if len(parts)-index > 1:
                            generator(parts[index+1:])
                        #reset
                        part.reset()
                elif part.dof.type == 'rule':
                    part.check_rule()
                    #print('rule checked', part.R_angle())

                if len(parts) == 1:
                    add_data_point()
                    part.reset()
                index += 1

        generator(self.dynamic_parts)
        return db_name

    def motionControl(self, target):
        for key, part in self.parts.items():
            part.reset()
        if self.db == None:
            print('Import or generate dataset')
            return
        angles, delta = self.db.find_match(target)
        print(angles)
        db_keys = self.db.keys()
        for key in db_keys:
            if key not in self.parts.keys():
                continue
            i = db_keys.index(self.parts[key].name)
            self.parts[key].rotate(self.parts[key].dof.axis, angles[i])
        return delta

    def plot(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for key, obj in self.parts.items():
            origin = np.squeeze(obj.ref_frame.mapFrom(obj.origin))
            endpoint = obj.mapFrom(obj.r)
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
    shoulder = Joint('shoulder', (0,0,0.5))
    shoulder.attach(base)
    shoulder.dynamic(type='rotation', axis='y', range=(-pi/2,pi/2))
    elbow = Joint('elbow', (0,0,0.5))
    elbow.attach(shoulder)
    elbow.dynamic(type='rotation', axis='y', range=(-pi/2,pi/2))
    wrist = Joint('wrist', (0,0,0.05))
    wrist.attach(elbow)
    wrist.dynamic(type='rotation', axis='y', range=(-pi/2,pi/2))
    suction = Joint('suction', (0,0,0.1))
    suction.attach(wrist)
    suction.dynamic(type='rule', axis='y', rule=(0,0,-1))
    # debug
    def debug():
        shoulder.rotate('y', 0.17453)
        elbow.rotate('y', 1.570796)
        wrist.rotate('y', 1.570796)
        #suction.rotate('y', -0.1745)
        suction.check_rule()
        print(suction.R_angle())
    arm.add(base)
    arm.add(shoulder)
    arm.add(elbow)
    arm.add(wrist)
    arm.add(suction)
    #debug()
    #arm.plot()
    #exit()
    return arm

arm = armSetup()
#arm.importDatabase('2019-10-07-205135.db')
arm.generateDatabase(10, debug=False, memory=True)
print(arm.motionControl((0.5,0,0.05)))
arm.plot()
