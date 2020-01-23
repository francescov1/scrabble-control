import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
from math import pi, sin, cos, sqrt
import sqlite3
from datetime import datetime
import os

from objects import Joint

DEBUG = False

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
        step_size = 2
        for axis in ['X','Y','Z']:
            filter += '({} BETWEEN {} AND {})'.format(axis, target[i]-step_size, target[i]+step_size)
            i += 1
            if i < 3:
                filter += ' AND '
        filter += ')'
        query = 'select {} from {} where {}'.format(col, Database.TBL_NAME, filter)
        result = self.c.execute(query)
        best_delta = float('Inf')
        best_row = None
        for row in result.fetchall():
            delta = sqrt((row[-3]-target[0])**2+(row[-2]-target[1])**2+(row[-1]-target[2])**2)
            if delta < best_delta:
                best_delta = delta
                best_row = row

        dict_row = {}
        return best_row, best_delta

    def open(self, path):
        self.conn = sqlite3.connect(path)
        #self.conn.row_factory = sqlite3.Row
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
        self.img_save_count = 0

    def importDatabase(self, file):
        self.db = Database(file)
        assert not bool(set([part.name for part in self.dynamic_parts]).intersection(set(self.db.keys())))

    def generateDatabase(self, resolution=2, debug=False, memory=False, bounding_box=None):
        default_bounds = (-float('Inf'),float('Inf'))
        if bounding_box == None:
            bounding_box = (default_bounds,default_bounds,default_bounds)
        else:
            bounding_box = list(bounding_box)
            for i in range(len(bounding_box)):
                if bounding_box[i] == None:
                    bounding_box[i] = default_bounds
            bounding_box = tuple(bounding_box)

        if bool(memory) or memory == None:
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

        def filter(endpoint):
            for i in range(3):
                if endpoint[i] < bounding_box[i][0] or endpoint[i] > bounding_box[i][1]:
                    return True
            return False

        def add_data_point():
            values = {}
            for part in self.dynamic_parts:
                if part.dof.type == 'rule':
                    values[part.name] = part.R_angle()[part.dof.axis]
                else:
                    values[part.name] = part.angle[part.dof.axis]
            endpoint = np.squeeze(part.mapFrom(part.r))
            if filter(endpoint):
                return
            values['X'] = endpoint[0]
            values['Y'] = endpoint[1]
            values['Z'] = endpoint[2]
            if debug:
                self.plot()
                if memory != None:
                    print(values)
            if memory != None:
                self.db.insert(values)

        def generator(parts):
            index=0
            for part in parts:
                if part.dof.type == 'rotation':
                    range = np.linspace(part.dof.range[0], part.dof.range[1], resolution)
                    for angle in range:
                        part.reset()
                        part.rotate(part.dof.axis, angle)
                        assert part.angle[part.dof.axis] == angle
                        if DEBUG:
                            print(part.name, 'rotated')
                        if len(parts)-index > 1:
                            generator(parts[index+1:])

                elif part.dof.type == 'rule':
                    part.reset()
                    part.check_rule()
                    if DEBUG:
                        print('rule checked', part.R_angle())

                if len(parts) == 1:
                    add_data_point()
                index += 1

        generator(self.dynamic_parts)
        if memory != None:
            return db_name

    def motionControl(self, target):
        for key, part in self.parts.items():
            part.reset()
        #translate - does not work in general, just when all joints share axis and translation along same axis (and only 1 translation rule)
        for key, part in self.parts.items():
            if part.dof.type == 'translation':
                new_origin = [0]*3
                axis_i = ['x','y','z'].index(part.dof.axis)
                new_origin[axis_i] = target[axis_i]
                part.translate(new_origin)
                break #Important: do not translate multiple times

        if self.db == None:
            print('Import or generate dataset')
            return
        angles, db_delta = self.db.find_match(target)
        if angles == None:
            print('Match not found')
            return
        db_keys = self.db.keys()
        for key in db_keys:
            if key not in self.parts.keys():
                continue
            i = db_keys.index(self.parts[key].name)
            self.parts[key].rotate(self.parts[key].dof.axis, angles[i])
        last_part = self.parts[self.db.keys()[-4]]
        r = last_part.mapFrom(last_part.r)
        delta = sqrt((target[0]-r[0])**2+(target[1]-r[1])**2+(target[2]-r[2])**2)
        assert abs(db_delta-delta) < 1
        return angles, delta

    def plot(self):
        fig = plt.figure()
        ax = fig.gca(projection='3d') #fig.add_subplot(111, projection='3d')
        for key, obj in self.parts.items():
            origin = np.squeeze(obj.ref_frame.mapFrom(obj.origin))
            endpoint = obj.mapFrom(obj.r)
            ax.plot(xs=(origin[0],endpoint[0]),
                    ys=(origin[1],endpoint[1]),
                    zs=(origin[2],endpoint[2]))

        lim = [-500,500]
        ax.set_xlim([0, max(lim)])
        ax.set_ylim([0, max(lim)])
        ax.set_zlim([0, max(lim)])

        ax.set_xlabel(r'$X$')
        ax.set_ylabel(r'$Y$')
        ax.set_zlabel(r'$Z$')
        ax.view_init(30,30)
        plt.savefig('img/{}.png'.format(self.img_save_count), dpi=96, transparent=False)
        self.img_save_count += 1
        plt.close('all')

    def add(self, object):
        if object.name in self.parts.keys():
            raise KeyError
        else:
            self.parts[object.name] = object
