#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Error calculated with the methods described from
    D. Prokhorov, D. Zhukov, O. Barinova, A. Vorontsova, and A. Konushin, “Measuring robustness of Visual SLAM,”
    arXiv:1910.04755 [cs], Oct. 2019, Accessed: Apr. 23, 2021. [Online]. Available: http://arxiv.org/abs/1910.04755
"""

from operator import index
import numpy as np
import matplotlib.pyplot as plt
import copy
from rospy.rostime import genpy

from src.trajectory import Trajectory
from src.quaternion import SLERP


class Error:
    def __init__(self, reference=None, estimate=None, delta=1):
        """Calculate Error(APE, RPE)
        APE 

        Args:
            reference (Trajectory): reference trajectory or ground truth trajectory. Defaults to None.
            estimate  (Trajectory): estimated trajectory for evaluation. Defaults to None.
            delta  (int, optional): local accuracy of the trajectory over a fixed time interval delta(for RPE). Defaults to 1
        """
        self.name = estimate.name
        print("Calculating {}'s Error with respect to Ground Truth Data".format(self.name))
        self.is_short = False

        self.reference, self.estimate = self._post_process(copy.deepcopy(reference), copy.deepcopy(estimate))
        self.time = self.estimate.time

        self.ape_trans, self.ape_rot = self.APE(self.reference, self.estimate)
        self.ape_tans_stat = self._statistics(self.ape_trans)
        self.ape_rot_stat = self._statistics(self.ape_rot)

        self.rpe_trans, self.rpe_rot = self.RPE(self.reference, self.estimate, delta)
        self.rpe_tans_stat = self._statistics(self.rpe_trans)
        self.rpe_rot_stat = self._statistics(self.rpe_rot)

    def _post_process(self, gt, test):
        orientation, trajectory, dur = [], [], []
        index = []
        for i in range(gt.length):
            time = gt.time[i]
            for j in range(test.length - 1):
                if test.time[j] < time < test.time[j + 1]:
                    alpha = (time - test.time[j]) / (test.time[j + 1] - test.time[j])
                    orientation.append(SLERP(test.orientation[j], test.orientation[j + 1], alpha))
                    trajectory.append((1 - alpha) * test.trajectory[j] + alpha * test.trajectory[j + 1])
                    dur.append(time)
                    index.append(i)
        index = np.array(index)

        gt.trajectory = gt.trajectory[index]
        gt.orientation = gt.orientation[index]
        gt.time = gt.time[index]
        gt.length = gt.trajectory.shape[0]

        test.trajectory = np.array(trajectory)
        test.orientation = np.array(orientation)
        test.time = np.array(dur)
        test.length = test.trajectory.shape[0]

        return gt, test

    def _statistics(self, error):
        std = np.std(error)
        mean = np.mean(error)
        median = np.median(error)
        minimum = np.min(error)
        maximum = np.max(error)
        rmse = np.sqrt((np.asarray(error) ** 2).mean())

        return [mean, std, median, minimum, maximum, rmse]

    def APE(self, gt, test):
        target_mean = gt.trajectory.mean(0)
        estimate_mean = test.trajectory.mean(0)

        target = gt.trajectory - target_mean
        estimate = test.trajectory - estimate_mean

        W = np.dot(target.T, estimate)
        U, _, V = np.linalg.svd(W, full_matrices=True, compute_uv=True)

        R = np.dot(U, V)
        t = target_mean - np.dot(R, estimate_mean)
        T = np.vstack([np.hstack([R, t.reshape(3, 1)]), np.array([0, 0, 0, 1])])

        ape_trans, ape_rot = [], []
        for i in range(gt.length):
            Q = gt.pose_matrix(i)
            P = test.pose_matrix(i)
            E = np.dot(np.linalg.inv(Q), np.dot(T, P))

            ape_trans.append(np.linalg.norm(E[:3, 3]))
            ape_rot.append(np.arccos((np.trace(E[:3, :3]) - 1) / 2))

        ''' direct comparison (no trajectory matching using Horn's method) 
        for i in range(gt.length):
            translation_error = np.linalg.norm(gt.trajectory[i] - test.trajectory[i])
            rotation_error = np.arccos((gt.orientation[i] ** -1 * test.orientation[i]).w) * 2 - np.pi
            ape_trans.append(translation_error)
            ape_rot.append(rotation_error)
        '''
        return ape_trans, ape_rot

    def RPE(self, gt, test, delta):
        rpe_trans, rpe_rot = [], []
        for i in range(gt.length - delta):
            Q = gt.pose_matrix(i)
            Q_delta = gt.pose_matrix(i+delta)
            Q = np.dot(np.linalg.inv(Q), Q_delta)
            P = test.pose_matrix(i)
            P_delta = test.pose_matrix(i+delta)
            P = np.dot(np.linalg.inv(P), P_delta)

            E = np.dot(np.linalg.inv(Q), P)

            rpe_trans.append(np.linalg.norm(E[:3, 3]))
            rpe_rot.append(np.arccos((np.trace(E[:3, :3]) - 1) / 2))
        ''' direct comparison (no conversion to pose matrix)
        for i in range(gt.length-delta):
            translation_error = np.linalg.norm((test.trajectory[i+delta]-test.trajectory[i]) - (gt.trajectory[i+delta] - gt.trajectory[i]))
            rotation_error = np.arccos((gt.orientation[i+delta]**-1 * gt.orientation[i] * test.orientation[i]**-1 * test.orientation[i+delta]).w) * 2 - np.pi
            rpe_trans.append(translation_error)
            rpe_rot.append(rotation_error)
        '''
        return rpe_trans, rpe_rot


def plotAPE(errors):
    plt.figure(figsize=(6, 5))
    plt.subplot(2, 1, 1)
    for error in errors:
        plt.plot(error.time, error.ape_trans, label=error.name)
        # for key, value in errors[i].ape_tans_stat.items():
        #     plt.axhline(y=value, color='r', linestyle='-', label=key)
    plt.legend()
    plt.title('APE')
    plt.ylabel('ape[m]')

    plt.subplot(2, 1, 2)
    for error in errors:
        plt.plot(error.time, error.ape_rot, label=error.name)
        # for key, value in errors[i].ape_tans_stat.items():
        #     plt.axhline(y=value, color='r', linestyle='-', label=key)
    plt.legend()
    plt.xlabel('time[nano_sec]')
    plt.ylabel('ape[rad]')


def plotRPE(errors):
    plt.figure(figsize=(6, 5))
    plt.subplot(2, 1, 1)
    for error in errors:
        plt.plot(error.time[1:], error.rpe_trans, label=error.name)
    plt.legend()
    plt.title('RPE')
    plt.ylabel('rpe[m]')

    plt.subplot(2, 1, 2)
    for error in errors:
        plt.plot(error.time[1:], error.rpe_rot, label=error.name)
    plt.legend()
    plt.xlabel('time[nano_sec]')
    plt.ylabel('rpe[rad]')


def plotAPEStats(errors):
    import pandas as pd
    index = ['mean', 'std', 'median', 'minimum', 'maximum', 'rmse']
    trans_dic = {}
    rot_dic = {}
    for error in errors:
        trans_dic[error.name] = error.ape_tans_stat
        rot_dic[error.name] = error.ape_rot_stat
    trans_data = pd.DataFrame(trans_dic, index=index)
    rot_data = pd.DataFrame(rot_dic, index=index)
    fig = plt.figure(figsize=(6, 5))

    ax = fig.add_subplot(2, 1, 1)
    trans_data.plot.barh(ax=ax)
    ax.title.set_text('APE Statistics')
    ax.set_xlabel('APE Translation')
    ax = fig.add_subplot(2, 1, 2)
    rot_data.plot.barh(ax=ax)
    ax.set_xlabel('APE Rotation')


def plotRPEStats(errors):
    import pandas as pd
    index = ['mean', 'std', 'median', 'minimum', 'maximum', 'rmse']
    trans_dic = {}
    rot_dic = {}
    for error in errors:
        trans_dic[error.name] = error.rpe_tans_stat
        rot_dic[error.name] = error.rpe_rot_stat
    trans_data = pd.DataFrame(trans_dic, index=index)
    rot_data = pd.DataFrame(rot_dic, index=index)

    fig = plt.figure(figsize=(6, 5))
    ax = fig.add_subplot(2, 1, 1)
    trans_data.plot.barh(ax=ax)
    ax.title.set_text('RPE Statistics')
    ax.set_xlabel('RPE Translation')
    ax = fig.add_subplot(2, 1, 2)
    rot_data.plot.barh(ax=ax)
    ax.set_xlabel('RPE Rotation')
