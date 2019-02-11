from numpy.random import randn
import scipy
from filterpy.kalman import KalmanFilter
import numpy as np
from scipy.linalg import block_diag
import matplotlib.pyplot as plt
from filterpy.common import Q_discrete_white_noise
from filterpy.stats import plot_covariance_ellipse


class PosSensor(object):
    def __init__(self, pos=(0.,0.,0.), vel=(0.,0.,0.), noise_std=1.):
        self.vel = [vel[0], vel[1], vel[2]]
        self.noise_std = noise_std
        self.pos = [pos[0], pos[1], pos[2]]


    def read(self):
        self.pos[0] += self.vel[0]
        self.pos[1] += self.vel[1] - 0.5*9.81
        self.pos[2] += self.vel[2]

        self.vel[1] = self.vel[1] - 9.81


        return [self.pos[0] + randn() * self.noise_std,
                self.pos[1] + randn() * self.noise_std,
                self.pos[2] + randn() * self.noise_std]

class TD2ndTracker:
    def __init__(self, R_std, Q_std, dt):
        self.tracker = KalmanFilter(dim_x=9, dim_z=3)
        # assign state variables here
        self.tracker.F = np.array([[1, 0, 0, dt, 0, 0, 0, 0, 0],
                               [0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0],
                               [0, 0, 1, 0, 0, dt, 0, 0, 0],
                               [0, 0, 0, 1, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, 1, 0, 0, dt, 0],
                               [0, 0, 0, 0, 0, 1, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0, 1, 0, 0],
                               [0, 0, 0, 0, 0, 0, 0, 1, 0],
                               [0, 0, 0, 0, 0, 0, 0, 0, 1]])

        # self.tracker.F = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
        #                            [0, 1, 0, 0, 0, 0, 0, 0, 0],
        #                            [0, 0, 1, 0, 0, 0, 0, 0, 0],
        #                            [0, 0, 0, 1, 0, 0, 0, 0, 0],
        #                            [0, 0, 0, 0, 1, 0, 0, 0, 0],
        #                            [0, 0, 0, 0, 0, 1, 0, 0, 0],
        #                            [0, 0, 0, 0, 0, 0, 1, 0, 0],
        #                            [0, 0, 0, 0, 0, 0, 0, 1, 0],
        #                            [0, 0, 0, 0, 0, 0, 0, 0, 1]])

        self.tracker.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0, 0, 0, 0]])

        self.tracker.R = np.eye(3) * R_std ** 2
        q = Q_discrete_white_noise(dim=3, dt=dt, var=Q_std ** 2)
        self.tracker.Q = scipy.linalg.block_diag(q, q, q)

        self.tracker.x = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0]]).T
        self.tracker.P = np.eye(9) * 0.5

    def update(self, z):
        self.tracker.update(z)

    def predict(self):
        return self.tracker.predict()



# from filterpy.kalman import KalmanFilter
# import numpy as np
# from scipy.linalg import block_diag
# import matplotlib.pyplot as plt
# from filterpy.common import Q_discrete_white_noise
# from filterpy.stats import plot_covariance_ellipse
#
# N = 15
# # number of iterations
# dt = 1.0 # time step
# R_std = 4
# Q_std = 0.04
#
# sensor = PosSensor((0, 0, 0), (2, 0, 2), noise_std=R_std)
# zs = np.array([sensor.read() for _ in range(N)])
# tracker3d = TD2ndTracker(R_std, Q_std, dt)
# xs, ys, zrs = [], [], []
# print(zs)
# for z in zs:
#     tracker3d.predict()
#     tracker3d.update(z)
#     xs.append(tracker3d.tracker.x[0])
#     ys.append(tracker3d.tracker.x[1])
#     zrs.append(tracker3d.tracker.x[2])
#     print("accelltimate: " + str(tracker3d.tracker.x[7]))
# plt.plot(xs, ys)
# sensor2 = PosSensor((0, 0, 0), (2, 0, 2), noise_std=R_std)
# x2s = []
# y2s = []
# for i in zs:
#     #x, y, z = sensor2.read()
#     x2s.append(i[0])
#     y2s.append(i[1])
# plt.plot(x2s, y2s, 'ro')
# plt.show()





