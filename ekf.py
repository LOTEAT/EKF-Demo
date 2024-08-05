'''
Author: LOTEAT
Date: 2024-08-01 10:59:21
'''
import numpy as np
from utils import normalize_angle


class EKFModel:

    def __init__(
        self,
        landmark_num=9,
    ):
        self.inf = 1e3
        self.landmark_num = landmark_num
        self.var_num = 2 * landmark_num + 3
        self.observerd_landmarks = []
        self.initialize()

    def move(self, pose, u):
        """move 

        Odometry motion model

        Args:
            pose (np.ndarray): the pose of the robot
            u (np.ndarray): control 

        Returns:
            pose: the pose after controlling
        """
        pose[0] += u['t'] * (np.cos(pose[2] + u['r1']))
        pose[1] += u['t'] * (np.sin(pose[2] + u['r1']))
        pose[2] = normalize_angle(pose[2] + u['r1'] + u['r2'])
        return pose

    def initialize(self):
        # initialize the mu, including the x,y coordinate of landmark and the pose of the robot
        self.mu = np.zeros(self.var_num)
        # the covariance of the robot
        robot_sigma = np.zeros((3, 3))
        # the covariance of the landmarks
        landmark_sigma = self.inf * np.eye(self.landmark_num * 2)
        robot_landmark_sigma = np.zeros((3, 2 * self.landmark_num))
        self.sigma = np.vstack([
            np.hstack([robot_sigma, robot_landmark_sigma]),
            np.hstack([robot_landmark_sigma.T, landmark_sigma])
        ])
        self.R = np.zeros((self.var_num, self.var_num))
        self.R[:3, :3] = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.01]])
        self.Q = 0.01

    def correction(self, observed_landmarks):
        observed_landmark_num = len(observed_landmarks)
        Z = np.zeros(2 * observed_landmark_num)
        Z_pred = np.zeros(2 * observed_landmark_num)
        H = np.zeros((0, self.var_num))
        for i, observed_landmark in enumerate(observed_landmarks):
            landmark_id = observed_landmark['id']
            index = int(landmark_id[-1])
            if landmark_id not in self.observerd_landmarks:
                self.observerd_landmarks.append(landmark_id)
                # only useful when the number of landmarks is less than 10
                self.mu[2 * index + 1:2 * index + 3] = self.mu[:2] + np.array([
                    observed_landmark['range'] *
                    np.cos(observed_landmark['bearing'] + self.mu[2]),
                    observed_landmark['range'] *
                    np.sin(observed_landmark['bearing'] + self.mu[2])
                ])
            Z[2 * i:2 * i + 2] = np.array(
                [observed_landmark['range'], observed_landmark['bearing']])
            delta = self.mu[2 * index + 1:2 * index + 3] - self.mu[:2]
            r = np.sqrt(delta.T @ delta)
            Z_pred[2 * i] = r
            Z_pred[2 * i + 1] = normalize_angle(
                np.arctan2(delta[1], delta[0]) - self.mu[2])
            Hi = np.vstack([
                np.array([
                    -delta[0] / r, -delta[1] / r, 0, delta[0] / r, delta[1] / r
                ]),
                np.hstack(
                    np.array([
                        delta[1] / np.square(r), -delta[0] / np.square(r), -1,
                        -delta[1] / np.square(r), delta[0] / np.square(r)
                    ]))
            ])
            mapping_mat = np.zeros((5, self.var_num))
            mapping_mat[0:3, 0:3] = np.eye(3)
            mapping_mat[3, 2 * index + 1] = 1
            mapping_mat[4, 2 * index + 2] = 1
            Hi = Hi @ mapping_mat
            H = np.vstack([H, Hi])
        Q = self.Q * np.eye(2 * observed_landmark_num)
        K = self.sigma @ H.T @ np.linalg.inv(H @ self.sigma @ H.T + Q)
        diff_Z = Z - Z_pred
        for i in range(1, diff_Z.shape[0], 2):
            diff_Z[i] = normalize_angle(diff_Z[i])
        self.mu = self.mu + K @ diff_Z
        self.sigma = (np.eye(self.var_num) - K @ H) @ self.sigma

    def prediction(self, u):
        G_robot = self.get_partial(self.mu[:3], u)
        self.mu[:3] = self.move(self.mu[:3], u)
        G = np.vstack([
            np.hstack([G_robot, np.zeros((3, 2 * self.landmark_num))]),
            np.hstack([
                np.zeros((2 * self.landmark_num, 3)),
                np.eye(2 * self.landmark_num)
            ])
        ])
        self.sigma = G @ self.sigma @ G.T + self.R

    def update(self, u, observed_landmarks):
        self.prediction(u)
        self.correction(observed_landmarks)

    def get_partial(self, pose, u):
        """get_partial

        Calculate the partial derivation of the odometry model.

        Args:
            u (dict): Control parameters.

        Returns:
            G (np.ndarray): Jacobian matrix.
        """
        G = np.eye(3)
        G[0, 2] -= u['t'] * np.sin(pose[2] + u['r1'])
        G[1, 2] += u['t'] * np.cos(pose[2] + u['r1'])
        return G

    def get_mu(self):
        return self.mu.copy()

    def get_sigma(self):
        return self.sigma.copy()
