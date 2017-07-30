"""
Module for helper functions for this library of simulation
"""
from math import pi
import numpy as np


class Pose2D:
    """
    Class to represent a 2D pose: (x, y, theta)
    """

    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

    def update(self, x, y, theta):
        """
        Update the pose and makes sure the angle is in [-pi,pi]
        :return:
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.theta = wrap_angles(self.theta)


def wrap_angles(angle):
    """
    Wrap angle between [-pi,pi]
    :param angle: angle to convert
    :return: converted angle
    """
    return (angle + pi) % (2 * pi) - pi


def homogeneous_transformation_matrix_2d(angle, tx, ty):
    """
    Create a transformation matrix (2D) to rotate and translate points in 2D
    (see section 3.4 of https://www.ensta-bretagne.fr/jaulin/mastersds_cours_robot_boimond.pdf)
    :param angle: rotation angle
    :param tx: x translation
    :param ty: y translation
    :return:
    """
    return np.array([[np.cos(angle), -np.sin(angle), tx],
                     [np.sin(angle), np.cos(angle), ty],
                     [0, 0, 1]])
