'''
Author: LOTEAT
Date: 2024-08-01 10:59:31
'''
import numpy as np


class OdometryMotion:

    def __init__(self, pose=np.zeros(3), noise=np.array([0.1, 0.1, 0.01])):
        self.pose = pose
        self.noise = noise

    def update(self, u):
        # odemetry motion model update
        self.pose[0] += u['t'] * (np.cos(self.pose[2] + u['r1']))
        self.pose[1] += u['t'] * (np.sin(self.pose[2] + u['r1']))
        self.pose[2] = self.pose[2] + u['r1'] + u['r2']

    def get_pose(self):
        return self.pose.copy()
    
    def set_pose(self, pose):
        self.pose = pose
        
    def get_partial(self, u):
        """get_partial

        Calculate the partial derivation of the odometry model.

        Args:
            u (dict): Control parameters.

        Returns:
            G (np.ndarray): Jacobian matrix.
        """
        G = np.eye(3)
        G[0, 2] -= u['t'] * np.sin(self.pose[2] + u['r1'])
        G[1, 2] += u['t'] * np.cos(self.pose[2] + u['r1'])
        return G

    def get_covariance(self):
        # return the covariance of motion noise
        return np.array([[self.noise[0], 0, 0], [0, self.noise[1], 0],
                         [0, 0, self.noise[2]]])
