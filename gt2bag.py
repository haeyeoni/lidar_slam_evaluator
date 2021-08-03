#! /usr/bin/env python
# -*- coding: utf-8 -*-

import math
import os
import sys
import argparse
import numpy as np

import pykitti
import rospy
import rosbag
from tqdm import tqdm
from tf.transformations import quaternion_from_matrix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

if __name__ == "__main__":
    seq_to_drive = {
        '00': '2011_10_03_drive_0027',
        '01': '2011_10_03_drive_0042',
        '02': '2011_10_03_drive_0034',
        '03': '2011_09_26_drive_0067',
        '04': '2011_09_30_drive_0016',
        '05': '2011_09_30_drive_0018',
        '06': '2011_09_30_drive_0020',
        '07': '2011_09_30_drive_0027',
        '08': '2011_09_30_drive_0028',
        '09': '2011_09_30_drive_0033',
        '10': '2011_09_30_drive_0034'
    }

    parser = argparse.ArgumentParser(description="Convert KITTI poses.txt to rosbag file!")
    parser.add_argument("-o", "--kitti_odom", help="KITTI odom dataset path", default="/home/dohoon/Datasets/kitti_odometry/dataset")
    parser.add_argument("-r", "--kitti_raw", help="KITTI raw dataset path", default="/home/dohoon/Datasets/kitti_raw/dataset")
    parser.add_argument("-s", "--sequence", help="sequence number", default="07")
    parser.add_argument("-p", "--path",  help="path to save a bag file", default="./bag")
    args = parser.parse_args()

    date = seq_to_drive[args.sequence][:-11]
    drive = seq_to_drive[args.sequence][17:]
    kitti_raw = pykitti.raw(args.kitti_raw, date, drive)
    kitti_odom = pykitti.odometry(args.kitti_odom, args.sequence)
    print('KITTI sequence {}({}) ready:'.format(args.sequence, seq_to_drive[args.sequence]))
    print('odom dataset from \'{}\''.format(args.kitti_odom))
    print('raw dataset from \'{}\''.format(args.kitti_raw))

    compression = rosbag.Compression.NONE

    save_path = os.path.join(args.path, "kitti_{}_synced".format(seq_to_drive[args.sequence]))
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    bag_fn = os.path.join(save_path, "kitti_gt.bag")
    bag = rosbag.Bag(bag_fn, 'w', compression=compression)
    gt_topic = '/ground_truth'

    path = Path()
    path.header.frame_id = '/map'
    R_transform = np.array([[0, 0, 1, 0],
                            [-1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, 0, 1]])
    q_transform = quaternion_from_matrix(R_transform)
    
    print('Conversion starts!')
    for i in tqdm(range(len(kitti_odom.timestamps))):
        q = quaternion_from_matrix(kitti_odom.poses[i])
        q = q / math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
        t = R_transform[:3, :3].dot(kitti_odom.poses[i][:3, 3].reshape(3, 1))
        pose = PoseStamped()
        pose.header.frame_id = '/map'
        pose.header.stamp = rospy.Time.from_sec(float(kitti_raw.timestamps[i].strftime("%s.%f")))
        pose.pose.position.x = float(t[0])
        pose.pose.position.y = float(t[1])
        pose.pose.position.z = float(t[2])
        pose.pose.orientation.x = q[2]
        pose.pose.orientation.y = -q[0]
        pose.pose.orientation.z = -q[1]
        pose.pose.orientation.w = q[3]
        path.header.stamp = pose.header.stamp
        path.poses.append(pose)
        bag.write(gt_topic, path, t=path.header.stamp)
    print('Finished.\n')

    print("## OVERVIEW ##")
    print(bag)
    bag.close()
