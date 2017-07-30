"""
Simulation of a sailboat based on L. Jaulin state equations (cf ROBMOOC and Mobile Robotics book)
"""
from helpers import Pose2D, homogeneous_transformation_matrix_2d
import numpy as np
from numpy.linalg import norm

# Parameters (see L. Jaulin book for definitions)
p1 = 0.1
p2 = 1.
p3 = 6000.
p4 = 1000.
p5 = 2000.
p6 = 1.
p7 = 1.
p8 = 2.
p9 = 300.
p10 = 10000.


class Sailboat:
    """
    Class to simulate the behaviour of a sailboat
    """

    # drawing points to represent a sailboat
    hull = np.array([[-1, 5, 7, 7, 5, -1, -1, -1],
                     [- 2, -2, -1, 1, 2, 2, -2, -2],
                     [1, 1, 1, 1, 1, 1, 1, 1]])
    sail = np.array([[-5, 0], [0, 0], [1, 1]])
    rudder = np.array([[-1, 1], [0, 0], [1, 1]])

    def __init__(self, pose=Pose2D(), v=0, w=0, dt=0.1):
        """
        The state vector of a sailboat is [x, y, theta, v, w]
        :param pose: initial 2D position
        :param v: initial speed
        :param w: initial rotation speed
        :param dt: simulation time step
        """
        self.dt = dt
        self.pose = pose
        # speed
        self.v = v
        # rotation speed
        self.w = w
        # -- we also need to keep track of the rudder and sail angles
        self.delta_r = 0
        self.delta_s = 0
        # -- not necessary but we can also track the rudder and sail force (for display or more)
        self.fr = 0
        self.fs = 0

    def xdot(self, delta_r, delta_s_max, wind_i, psi):
        """
        xdot is the derivative of the state vector (xdot = dx/dt)
        :param delta_r: rudder angle (command)
        :param delta_s_max: variation of the sail max allowed opening (command)
        :param wind_i: wind intensity
        :param psi: wind direction
        :return: xdot
        """
        self.delta_r = delta_r
        w_ap = np.array([wind_i * np.cos(psi - self.pose.theta) - self.v,
                         wind_i * np.sin(psi - self.pose.theta)])
        psi_ap = np.arctan2(w_ap[1], w_ap[0])
        a_ap = norm(w_ap)
        sigma = np.cos(psi_ap) + np.cos(delta_s_max)
        if sigma < 0:
            self.delta_s = np.pi + psi_ap
        else:
            self.delta_s = -np.sign(np.sin(psi_ap)) * delta_s_max
        print np.sin(self.delta_s - psi_ap), a_ap, p4
        self.fr = p5 * self.v * np.sin(delta_r)
        self.fs = p4 * a_ap * np.sin(self.delta_s - psi_ap)
        dx = self.v * np.cos(self.pose.theta) + p1 * wind_i * np.cos(psi)
        dy = self.v * np.sin(self.pose.theta) + p1 * wind_i * np.sin(psi)
        dtheta = self.w
        dv = (1. / p9) * (np.sin(self.delta_s) * self.fs -
                          np.sin(delta_r) * self.fr - p2 * self.v ** 2)
        dw = (1. / p10) * ((p6 - p7 * np.cos(self.delta_s)) * self.fs -
                           p8 * np.cos(delta_r) * self.fr -
                           p3 * self.w * self.v)
        Xdot = np.array([dx, dy, dtheta, dv, dw])
        return Xdot

    def simulate(self, delta_r, delta_s_max, wind_i, psi):
        """
        Simulate the sailboat with euler method
        (see xdot method for parameter definitions)
        """
        X = [self.pose.x, self.pose.y, self.pose.theta, self.v, self.w]
        X = np.array(X) + self.xdot(delta_r, delta_s_max, wind_i, psi) * self.dt
        self.pose.update(X[0], X[1], X[2])
        self.v = X[3]
        self.w = X[4]

    def __repr__(self):
        return 'Car position is ({:.2f},{:.2f}) and rotated at {:.2f} degrees'.format(self.pose.x, self.pose.y,
                                                                                      np.degrees(self.pose.theta))

    def mpl_display(self, ax, draw_forces=False):
        """
        Draws the sailboat in the given matplotlib axes.
        :param ax: matplotlib figure axes
        :param draw_forces: draw_forces if True
        """
        x, y, theta = self.pose.x, self.pose.y, self.pose.theta  # easier to read
        # ---- Sailboat drawing
        # Hull rotation
        rot = homogeneous_transformation_matrix_2d(theta, x, y)
        img_hull = np.dot(rot, self.hull)
        ax.plot(img_hull[0], img_hull[1], 'k')
        # Sail rotation
        rot_sail = homogeneous_transformation_matrix_2d(self.delta_s, 3, 0)
        img_sail = rot.dot(rot_sail).dot(self.sail)
        ax.plot(img_sail[0], img_sail[1], 'k')
        # Rudder rotation
        rot_rudder = homogeneous_transformation_matrix_2d(self.delta_r, -1, 0)
        img_rudder = rot.dot(rot_rudder).dot(self.rudder)
        ax.plot(img_rudder[0], img_rudder[1], 'k')
        # ---- Forces drawing
        if draw_forces:
            # force on sail
            Mfs = np.array([[-1., -1.], [0, -self.fs / 1000.], [1., 1.]])
            Mfs = np.dot(np.dot(rot, rot_sail), Mfs)
            ax.plot(Mfs[0], Mfs[1], 'r')
            # force on rudder
            Mfr = np.array([[0., 0.], [0., self.fr / 100.], [1., 1.]])
            Mfr = np.dot(np.dot(rot, rot_rudder), Mfr)
            ax.plot(Mfr[0], Mfr[1], 'b')


if __name__ == '__main__':
    from numpy import arange
    # Simulation parameters
    T = 10  # seconds
    dt = 0.1
    # Model instantiation
    sailboat = Sailboat()
    for step in arange(0, T, dt):
        sailboat.simulate(0.1, 1, 2, np.pi/3)
        print sailboat
