"""
Simulation of a car
"""
from helpers import Pose2D, homogeneous_transformation_matrix_2d
from math import sin, cos, degrees
import numpy as np


class Car(object):
    """
    Car model simulation class. In this model, we control the steering speed(d theta/dt) of the car only.
    """

    # simple drawing points to represent a car
    img = np.array([[1, -1, 0, 0, -1, -1, 0, 0, -1, 1, 0, 0, 3, 3, 0],
                    [-2, -2, -2, -1, -1, 1, 1, 2, 2, 2, 2, 1, 0.5, -0.5, -1]])
    img = np.vstack((img, np.ones(img.shape[1])))

    def __init__(self, pose=Pose2D(), dt=0.1, speed=2):
        """
        A car is defined by its pose and speed.
        :param pose: initial position of the car
        :param dt: euler simulation step
        :param speed: car speed
        """
        self.pose = pose
        self.speed = speed
        self.dt = dt

    def simulate(self, cmd):
        """
        simulate the car for an euler time step.
        :param cmd: steering command (belongs to [-pi, pi]) (rad/s)
        """
        x = self.pose.x + self.speed * self.dt * cos(self.pose.theta)
        y = self.pose.y + self.speed * self.dt * sin(self.pose.theta)
        theta = self.pose.theta + cmd * self.dt
        self.pose.update(x, y, theta)

    def __repr__(self):
        return 'Car position is ({:.2f},{:.2f}) and rotated at {:.2f} degrees'.format(self.pose.x, self.pose.y,
                                                                                      degrees(self.pose.theta))

    def mpl_display(self, ax):
        """
        Plot the robot position in matplotlib axes
        :return:
        """
        x, y, theta = self.pose.x, self.pose.y, self.pose.theta     # easier to read
        print x, y
        # transformation matrix
        rotM = homogeneous_transformation_matrix_2d(theta, x, y)
        # rotate image
        img_res = np.dot(rotM, self.img)
        # plot in matplotlib figure
        ax.plot(img_res[0], img_res[1])


if __name__ == '__main__':
    from numpy import arange
    # Simulation parameters
    T = 10  # seconds
    dt = 0.1
    car = Car(Pose2D(), dt)
    for step in arange(0, T, dt):
        car.simulate(0.1)
        print car
