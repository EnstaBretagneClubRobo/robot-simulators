#!/usr/bin/env python
"""
Example of ROS Node to simulate a car.
The node subscribes to the command topic to update the command.
A loop simulates the robot with the last received command and publishes its position.
"""
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from robot_models import Car


# --------------------------------------------------------------------------------
# Initialize ROS node
# --------------------------------------------------------------------------------
rospy.init_node('car_sim')

# --------------------------------------------------------------------------------
# Simulation parameters
# --------------------------------------------------------------------------------
# total simulation time
T = 10
# simulation step
dt = 0.1
freq = 1/dt

# --------------------------------------------------------------------------------
# Subscribe to the command topic
# --------------------------------------------------------------------------------
cmd = 0


def update_cmd(msg):
    global cmd
    cmd = msg.data

rospy.Subscriber('/cmd', Float64, update_cmd)

# --------------------------------------------------------------------------------
# Setup pose publisher
# --------------------------------------------------------------------------------
pose_pub = rospy.Publisher('/car_pose', Pose2D, queue_size=1)

# --------------------------------------------------------------------------------
# Instantiate the car model
# --------------------------------------------------------------------------------
car = Car(dt=dt)

# --------------------------------------------------------------------------------
# Simulation Loop
# --------------------------------------------------------------------------------

rate = rospy.Rate(freq)

while not rospy.is_shutdown():
    # simulate the car
    car.simulate(cmd)
    # build the pose message
    pose_msg = Pose2D()
    pose_msg.x = car.pose.x
    pose_msg.y = car.pose.y
    pose_msg.theta = car.pose.theta
    # publish the msg
    pose_pub.publish(pose_msg)
    rate.sleep()
