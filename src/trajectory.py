import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import rosbag

import src.quaternion as quat


class Trajectory:
    def __init__(self, file_name):
        """Get trajectory and pose data from a file

        Args:
            file_name (string): data's file name 
        """
        print("Reading {}".format(file_name))
        self.is_None = False
        if file_name.endswith('.txt') or file_name.endswith('.csv'):
            pose = pd.read_csv(file_name, sep=' ',
                               names=['x1', 'x2', 'x3', 'x4', 'x5', 'x6', 'x7', 'x8', 'x9', 'x10', 'x11', 'x12'])
            name = file_name.split('/')[2].split('.')[0].split('_')[2]
            time = None
            length = pose.shape[0]
        elif file_name.endswith('.bag'):
            with rosbag.Bag(file_name) as bag:
                trajectory, orientation, name, time_dur, length = self._read_bag(bag)
        else:
            print("unsupported type of data file")
            self.is_None = True

        self.orientation = np.array(orientation)
        self.trajectory = np.array(trajectory)

        self.time = np.array(time_dur)
        self.name = name
        self.length = length

        self.is_gt = False
        if self.name == 'gt' or self.name == 'ground_truth' or self.name == '/ground_truth': self.is_gt = True
        print("{} with length {}".format(self.name, self.length))

    def _read_bag(self, bag):
        traj, rot, time = [], [], []
        for topic, msg, _ in bag.read_messages():
            poses = msg.poses
        for msg in poses:
            traj.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            rot.append(quat.Quaternion([msg.pose.orientation.x, msg.pose.orientation.y,
                                        msg.pose.orientation.z, msg.pose.orientation.w]))
            time.append((msg.header.stamp - poses[0].header.stamp).to_nsec())
        return traj, rot, topic, time, bag.get_message_count()

    def pose_matrix(self, index):
        return np.vstack([np.hstack([self.orientation[index].rotation(), self.trajectory[index].reshape(3, 1)]),
                          np.array([0, 0, 0, 1])])

def plotXYZ(gt, trajs):
    plt.figure(figsize=(6, 10))
    plt.subplot(3, 1, 1)
    for traj in trajs:
        plt.plot(traj.time, traj.trajectory[:, 0], label=traj.name)
    if gt: plt.plot(gt.time, gt.trajectory[:, 0], label=gt.name, ls='--')
    plt.ylabel('x[m]')
    plt.legend()

    plt.subplot(3, 1, 2)
    for traj in trajs:
        plt.plot(traj.time, traj.trajectory[:, 1], label=traj.name)
    if gt: plt.plot(gt.time, gt.trajectory[:, 1], label=gt.name, ls='--')
    plt.ylabel('y[m]')
    plt.legend()

    plt.subplot(3, 1, 3)
    for traj in trajs:
        plt.plot(traj.time, traj.trajectory[:, 2], label=traj.name)
    if gt: plt.plot(gt.time, gt.trajectory[:, 2], label=gt.name, ls='--')
    plt.ylabel('z[m]')
    plt.xlabel('time[nano_sec]')
    plt.legend()


def plot2D(option, gt, trajs):
    plt.figure(figsize=(6, 5))
    plt.title('Top-View')
    if option == 'xy':
        for traj in trajs:
            plt.plot(traj.trajectory[:, 0], traj.trajectory[:, 1], label=traj.name)
        if gt: plt.plot(gt.trajectory[:, 0], gt.trajectory[:, 1], label=gt.name, ls='--')
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
    if option == 'xz':
        for traj in trajs:
            plt.plot(traj.trajectory[:, 0], traj.trajectory[:, 2], label=traj.name)
        if gt: plt.plot(gt.trajectory[:, 0], gt.trajectory[:, 2], label=gt.name, ls='--')
        plt.xlabel("x[m]")
        plt.ylabel("z[m]")
        plt.legend()


def plot3D(gt, trajs):
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure(figsize=(6, 5))
    ax = fig.add_subplot(111, projection='3d')
    for traj in trajs:
        ax.scatter(traj.trajectory[:, 0], traj.trajectory[:, 1], traj.trajectory[:, 2], label=traj.name)
    if gt: ax.scatter(gt.trajectory[:, 0], gt.trajectory[:, 1], gt.trajectory[:, 2], label=gt.name)
    ax.legend()
    ax.set_zlim3d(-40, 40)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
