'''
Author: LOTEAT
Date: 2024-07-31 11:23:28
'''
import numpy as np


def read_sensor_data(file_path):
    """ 

    Read odometry data and sensor data from `file_path`

    Args:
        file_path (str): The path to the file.

    Returns:
        list: A list of dictionaries containing the sensor data and odometry data.
    """
    data = []
    with open(file_path, 'r', encoding='utf-8') as f:
        raw_data = f.readlines()
    odometry = dict()
    sensor = []
    is_complete = False
    for each_data in raw_data:
        if 'ODOMETRY' in each_data:
            if is_complete:
                snapshot = dict(odometry=odometry, sensor=sensor)
                data.append(snapshot)
                is_complete = False
            odometry_data = each_data.strip().split(' ')[1:]
            odometry = dict(
                r1=float(odometry_data[0]),
                t=float(odometry_data[1]),
                r2=float(odometry_data[2]),
            )
            sensor = []
            is_complete = True
        else:
            sensor_data = each_data.strip().split(' ')
            sensor_id, sensor_range, sensor_bearing = sensor_data
            sensor.append(
                dict(id=sensor_id,
                     range=float(sensor_range),
                     bearing=float(sensor_bearing)))
    return data


def read_world(file_path):
    """ 

    Read landmark data from `file_path`

    Args:
        file_path (str): The path to the file.

    Returns:
        dict: A dictionary containing the coordinate of landmarks.
    """
    landmarks = dict()
    with open(file_path, 'r', encoding='utf-8') as f:
        raw_data = f.readlines()
    for each_data in raw_data:
        landmark = each_data.strip().split(' ')
        landmarks[landmark[0]] = (float(landmark[1]), float(landmark[2]))
    return landmarks


def normalize_angle(angle):
    """normalize_angle 

    Normalize angle to be between -pi and pi

    Args:
        angle (numpy.ndarray): angle
    """
    while angle > np.pi:
        angle = angle - 2 * np.pi

    while angle < -np.pi:
        angle = angle + 2 * np.pi
    return angle
