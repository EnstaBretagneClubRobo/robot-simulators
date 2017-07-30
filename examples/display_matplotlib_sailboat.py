"""
Example to display the simulation of a model with matplotlib
Note that matplotlib is not great at real-time display. So it's just a tool to see if the behaviour is correct.
"""
import matplotlib.pyplot as plt
from robot_models import Car
from robot_models.helpers import Pose2D
import numpy as np

# --------------------------------------------------------------------------------
# Simulation parameters
# --------------------------------------------------------------------------------
# total simulation time
T = 10
# simulation step
dt = 0.1

# --------------------------------------------------------------------------------
# Creation of the figure
# --------------------------------------------------------------------------------
fig = plt.figure('Car simulation')
ax = fig.add_subplot(111)

# --------------------------------------------------------------------------------
# Instantiate the car model
# --------------------------------------------------------------------------------
car = Car(Pose2D(theta=1.2), 0.1)

# --------------------------------------------------------------------------------
# Simulation
# --------------------------------------------------------------------------------
for step in np.arange(0, T, dt):
    ax.cla()
    car.simulate(0.1)
    car.mpl_display(ax)
    ax.axis([-10, 10, -10, 10])
    plt.pause(dt)

plt.show()

