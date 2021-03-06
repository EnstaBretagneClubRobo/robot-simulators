"""
Example to display the simulation of a sailboat with matplotlib
Note that matplotlib is not great at real-time display. So it's just a tool to see if the behaviour is correct.
"""
import matplotlib.pyplot as plt
from robot_models import Sailboat
from robot_models.helpers import Pose2D
import numpy as np

# --------------------------------------------------------------------------------
# Simulation parameters
# --------------------------------------------------------------------------------
# total simulation time
T = 30
# simulation step
dt = 0.1

# --------------------------------------------------------------------------------
# Creation of the figure
# --------------------------------------------------------------------------------
fig = plt.figure('Sailboat simulation')
ax = fig.add_subplot(111)

# --------------------------------------------------------------------------------
# Instantiate the car model
# --------------------------------------------------------------------------------
sailboat = Sailboat(Pose2D(theta=np.pi), v=5, dt=dt)

# --------------------------------------------------------------------------------
# Simulation
# --------------------------------------------------------------------------------
for step in np.arange(0, T, dt):
    ax.cla()
    sailboat.simulate(0.1, np.pi/3, 2, np.pi/2)
    sailboat.mpl_display(ax, draw_forces=True)
    ax.axis([-50, 50, -50, 50])
    plt.pause(0.001)

plt.show()

