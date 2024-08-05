'''
Author: LOTEAT
Date: 2024-07-31 15:47:31
'''

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import PillowWriter
import threading
import numpy as np
from scipy.stats import chi2


def get_probellipse_args(x, C, alpha):
    sxx = C[0, 0]
    syy = C[1, 1]
    sxy = C[0, 1]
    a = np.sqrt(0.5 * (sxx + syy + np.sqrt((sxx - syy)**2 + 4 * sxy**2)))
    b = np.sqrt(0.5 * (sxx + syy - np.sqrt((sxx - syy)**2 + 4 * sxy**2)))
    a = np.real(a)
    b = np.real(b)

    chi2_val = chi2.ppf(alpha, 2)
    a *= np.sqrt(chi2_val)
    b *= np.sqrt(chi2_val)

    if sxx < syy:
        a, b = b, a

    if sxx != syy:
        angle = 0.5 * np.arctan2(2 * sxy, (sxx - syy))
    elif sxy == 0:
        angle = 0
    elif sxy > 0:
        angle = np.pi / 4
    else:
        angle = -np.pi / 4
    args = dict(center=(x[0], x[1]),
                width=2 * a,
                height=2 * b,
                angle=np.degrees(angle))
    return args


def update_probellipse(ellipse, args):
    ellipse.set_center(args['center'])
    ellipse.set_width(args['width'])
    ellipse.set_height(args['height'])
    ellipse.set_angle(args['angle'])
    ellipse.set_visible(True)


def plot_motion(trajectory, landmarks):
    fig, ax = plt.subplots()
    ax.set_xlim(-2, 12)
    ax.set_ylim(-2, 12)
    landmarks_x = []
    landmarks_y = []
    for landmark_coord in landmarks.values():
        landmarks_x.append(landmark_coord[0])
        landmarks_y.append(landmark_coord[1])
    ax.scatter(landmarks_x, landmarks_y, c='red', label='Landmarks')

    robot, = ax.plot([], [], 'bo', label='Robot')
    connections, = ax.plot([], [], 'k-')
    ellipses = [
        plt.matplotlib.patches.Ellipse((0, 0),
                                       width=0,
                                       height=0,
                                       angle=0,
                                       edgecolor='red',
                                       fc='None',
                                       lw=2)
    ]
    ellipses = ellipses + [
        plt.matplotlib.patches.Ellipse((0, 0),
                                       width=0,
                                       height=0,
                                       angle=0,
                                       edgecolor='blue',
                                       fc='None',
                                       lw=2)
    for _ in range(9)] 
    for ellipse in ellipses:
        ax.add_patch(ellipse)

    def init():
        robot.set_data([], [])
        connections.set_data([], [])
        for ellipse in ellipses:
            ellipse.set_visible(False)
        return [robot, connections] + ellipses

    def update(frame):
        mu = frame['mu']
        sigma = frame['sigma']
        pose = mu[:2]
        sensors = frame['sensor']
        # ellipse = draw_probellipse(mu[:2], sigma[:2, :2], alpha=0.95, color='blue')
        robot_ellipse = ellipses[0]
        robot_ellipse_args = get_probellipse_args(mu[:2],
                                                  sigma[:2, :2],
                                                  alpha=0.95)
        update_probellipse(robot_ellipse, robot_ellipse_args)

        robot.set_data(pose[0], pose[1])
        x_data = []
        y_data = []
        for sensor_data in sensors:
            landmark_id = sensor_data['id']
            landmark_coord = landmarks[landmark_id]
            x_data.extend([pose[0], landmark_coord[0]])
            y_data.extend([pose[1], landmark_coord[1]])
            ellipse_index = int(landmark_id[-1]) - 1
            # input(ellipse_index)
            landmark_ellipse = ellipses[ellipse_index + 1]
            landmark_ellipse_args = get_probellipse_args(
                mu[2 * ellipse_index + 3:2 * ellipse_index + 5],
                sigma[2 * ellipse_index + 3:2 * ellipse_index + 5,
                      2 * ellipse_index + 3:2 * ellipse_index + 5],
                alpha=0.95)
            update_probellipse(landmark_ellipse, landmark_ellipse_args)

        connections.set_data(x_data, y_data)

        if frame == trajectory[-1]:
            threading.Timer(1, plt.close, [fig]).start()

        return [robot, connections] + ellipses

    ani = animation.FuncAnimation(fig,
                                  update,
                                  frames=trajectory,
                                  init_func=init,
                                  blit=True,
                                  interval=50,
                                  repeat=False)
    ax.legend()
    plt.show()
    writer = PillowWriter(fps=20)
    ani.save("motion.gif", writer=writer)
