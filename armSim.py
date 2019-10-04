import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
from math import pi, sin, cos, sqrt
import sklearn
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.ensemble import RandomForestRegressor
from sklearn.metrics import mean_squared_error

from objects import Joint

class Arm:
    def __init__(self):
        self.parts = {}
        self.model = None
        self.dynamic_parts = []
        self.trained = False

    def generateModel(self, X, Y):
        #X_train, X_test, Y_train, Y_test = train_test_split(X,Y, test_size=0.1, random_state=5)
        model = RandomForestRegressor(n_estimators=50, max_features=3)
        model.fit(X, Y)
        #https://towardsdatascience.com/linear-regression-on-boston-housing-dataset-f409b7e4a155
        # model evaluation for training set
        #y_train_predict = model.predict(X_train)
        #rmse = (np.sqrt(mean_squared_error(Y_train, y_train_predict)))
        #print("The model performance for training set")
        #print("--------------------------------------")
        #print('RMSE is {}'.format(rmse))
        #print("\n")
        # model evaluation for testing set
        #y_test_predict = model.predict(X_test)
        #rmse = (np.sqrt(mean_squared_error(Y_test, y_test_predict)))
        #print("The model performance for testing set")
        #print("--------------------------------------")
        #print('RMSE is {}'.format(rmse))
        return model

    def learn(self, dataset):
        Y = dataset[:,0:-3]
        X = dataset[:,-4:-1]
        self.model = self.generateModel(X,Y)
        self.trained = True

    def generateTraining(self, resolution=2):
    #only for parts that rotate
        for key, part in self.parts.items():
            if not part.dof.fixed and part.dof.type in ['rotation', 'rule']:
                self.dynamic_parts.append(part)

        dataset_width = len(self.dynamic_parts) + 3
        dataset = np.matrix([part.name for part in self.dynamic_parts]+['X','Y','Z'])
        dataset.reshape((1, dataset_width))

        def add_data_point():
            nonlocal dataset
            dpt = np.zeros(shape=(1, dataset_width))
            for i in range(len(self.dynamic_parts)):
                dpt[0][i] = self.dynamic_parts[i].angle[self.dynamic_parts[i].dof.axis]
            endpoint = np.squeeze(self.dynamic_parts[-1].mapFrom(self.dynamic_parts[-1].r).reshape(1,-1))
            dpt[0][-3:-1] = endpoint[-3:-1]
            #self.plot() #For debugging
            dataset = np.append(dataset, dpt, axis=0)

        def generator(parts):
            index = 0
            for part in parts:
                if part.dof.type == 'rule':
                    part.check_rule()
                    break
                range = np.linspace(part.dof.range[0], part.dof.range[1], resolution)
                for angle in range:
                    part.rotate(part.dof.axis, angle)
                    if len(parts)-index > 1:
                        generator(parts[1:])
                    add_data_point()
                    #reset
                    part.reset()
                index+=1
        generator(self.dynamic_parts)
        dataset = dataset[1:].astype(np.float64)
        np.savetxt('training_data', dataset)
        return dataset

    def motionControl(self, target):
        if not self.trained:
            print('Arm must be trained first')
            return
        target = np.array(target).reshape(1,3)
        angles = np.squeeze(self.model.predict(target))
        for i in range(len(angles)):
            part = self.dynamic_parts[i]
            print(angles[i])
            part.rotate(axis=part.dof.axis, angle=angles[i])

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
    shoulder.attach(base)
    shoulder.dynamic(type='rotation', axis='y', range=(-pi/3,pi/2))
    elbow = Joint('elbow',(0.75,0,0))
    elbow.attach(shoulder)
    elbow.dynamic(type='rotation', axis='y', range=(-pi/2,pi/2))
    wrist = Joint('wrist', (0.25,0,0))
    wrist.attach(elbow)
    wrist.dynamic(type='rule', axis='y', rule=(0,0,-1))
    wrist.check_rule()
    wrist.reset()
    arm.add(base)
    arm.add(shoulder)
    arm.add(elbow)
    arm.add(wrist)

    return arm

arm = armSetup()
training_data = arm.generateTraining(50)
arm.learn(training_data)
arm.motionControl((0.5,0,0.75))
wrist = arm.parts['wrist']
print(wrist.mapFrom(wrist.r))
arm.plot()
