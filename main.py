'''
Author: LOTEAT
Date: 2024-07-29 19:31:14
'''
from utils import read_sensor_data, read_world
from ekf import EKFModel
from plot import plot_motion

data = read_sensor_data('data/sensor_data.txt')
landmarks = read_world('data/world.txt')
robot = EKFModel()
trajectory = [dict(sensor=[], mu=robot.get_mu(), sigma=robot.get_sigma())]

for snapshot in data:
    odometry = snapshot['odometry']
    sensor = snapshot['sensor']
    robot.update(odometry, sensor)
    trajectory.append(dict(sensor=sensor, mu=robot.get_mu(), sigma=robot.get_sigma()))

plot_motion(trajectory, landmarks)