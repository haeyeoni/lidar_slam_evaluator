# MIT License
#
# Copyright (c) 2021 Dohoon Cho
# Copyright (c) 2016-2020 Tomas Krejci
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#!env python
# -*- coding: utf-8 -*-

import sys
import os
import argparse
from tqdm import tqdm
from datetime import datetime

try:
    import pykitti
except ImportError as e:
    print('Could not load module \'pykitti\'. Please run `pip install pykitti`')
    sys.exit(1)

import tf
import rospy
import rosbag
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TwistStamped
import numpy as np

def save_imu_data(bag, kitti, imu_frame_id, topic):
    print("Exporting IMU")
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        q = tf.transformations.quaternion_from_euler(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        imu.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = oxts.packet.af
        imu.linear_acceleration.y = oxts.packet.al
        imu.linear_acceleration.z = oxts.packet.au
        imu.angular_velocity.x = oxts.packet.wf
        imu.angular_velocity.y = oxts.packet.wl
        imu.angular_velocity.z = oxts.packet.wu
        bag.write(topic, imu, t=imu.header.stamp)

def save_imu_data_raw(bag, kitti, imu_frame_id, topic):
    print("Exporting IMU Raw")
    synced_path = kitti.data_path
    unsynced_path = synced_path.replace('sync', 'extract')
    imu_path = os.path.join(unsynced_path, 'oxts')

    # read time stamp (convert to ros seconds format)
    with open(os.path.join(imu_path, 'timestamps.txt')) as f:
        lines = f.readlines()
        imu_datetimes = []
        for line in lines:
            if len(line) == 1:
                continue
            timestamp = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            imu_datetimes.append(float(timestamp.strftime("%s.%f")))

    # fix imu time using a linear model (may not be ideal, ^_^)
    imu_index = np.asarray(range(len(imu_datetimes)), dtype=np.float64)
    z = np.polyfit(imu_index, imu_datetimes, 1)
    imu_datetimes_new = z[0] * imu_index + z[1]
    imu_datetimes = imu_datetimes_new.tolist()

    # get all imu data
    imu_data_dir = os.path.join(imu_path, 'data')
    imu_filenames = sorted(os.listdir(imu_data_dir))
    imu_data = [None] * len(imu_filenames)
    for i, imu_file in enumerate(imu_filenames):
        imu_data_file = open(os.path.join(imu_data_dir, imu_file), "r")
        for line in imu_data_file:
            if len(line) == 1:
                continue
            stripped_line = line.strip()
            line_list = stripped_line.split()
            imu_data[i] = line_list

    assert len(imu_datetimes) == len(imu_data)
    
    for timestamp, data in tqdm(zip(imu_datetimes, imu_data)):
        roll, pitch, yaw = float(data[3]), float(data[4]), float(data[5]), 
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        imu.header.stamp = rospy.Time.from_sec(timestamp)
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = float(data[11])
        imu.linear_acceleration.y = float(data[12])
        imu.linear_acceleration.z = float(data[13])
        imu.angular_velocity.x = float(data[17])
        imu.angular_velocity.y = float(data[18])
        imu.angular_velocity.z = float(data[19])
        bag.write(topic, imu, t=imu.header.stamp)

        imu.header.frame_id = 'imu_enu_link'
        bag.write('/imu_correct', imu, t=imu.header.stamp) # for LIO-SAM GPS

        
def save_velo_data(bag, kitti, velo_frame_id, topic):
    print("Exporting velodyne data")
    velo_path = os.path.join(kitti.data_path, 'velodyne_points')
    velo_data_dir = os.path.join(velo_path, 'data')
    velo_filenames = sorted(os.listdir(velo_data_dir))
    with open(os.path.join(velo_path, 'timestamps.txt')) as f:
        lines = f.readlines()
        velo_datetimes = []
        for line in lines:
            if len(line) == 1:
                continue
            dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            velo_datetimes.append(dt)

    iterable = zip(velo_datetimes, velo_filenames)

    count = 0

    for dt, filename in tqdm(iterable, total=len(velo_filenames)):
        if dt is None:
            continue

        velo_filename = os.path.join(velo_data_dir, filename)

        # read binary data
        scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)

        # get ring channel
        depth = np.linalg.norm(scan, 2, axis=1)
        pitch = np.arcsin(scan[:, 2] / depth) # arcsin(z, depth)
        fov_down = -24.8 / 180.0 * np.pi
        fov = (abs(-24.8) + abs(2.0)) / 180.0 * np.pi
        proj_y = (pitch + abs(fov_down)) / fov  # in [0.0, 1.0]
        proj_y *= 64  # in [0.0, H]
        proj_y = np.floor(proj_y)
        proj_y = np.minimum(64 - 1, proj_y)
        proj_y = np.maximum(0, proj_y).astype(np.int32)  # in [0,H-1]
        proj_y = proj_y.reshape(-1, 1)
        scan = np.concatenate((scan,proj_y), axis=1)
        scan = scan.tolist()
        for i in range(len(scan)):
            scan[i][-1] = int(scan[i][-1])

        # create header
        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = rospy.Time.from_sec(float(datetime.strftime(dt, "%s.%f")))

        # fill pcl msg
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1),
                  PointField('ring', 16, PointField.UINT16, 1)]
        pcl_msg = pcl2.create_cloud(header, fields, scan)
        pcl_msg.is_dense = True
        # print(pcl_msg)

        bag.write(topic, pcl_msg, t=pcl_msg.header.stamp)

        # count += 1
        # if count > 200:
        #     break


def inv(transform):
    "Invert rigid body transformation matrix"
    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = t_inv
    return transform_inv


def save_gps_fix_data(bag, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.frame_id = gps_frame_id
        navsatfix_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        navsatfix_msg.latitude = oxts.packet.lat
        navsatfix_msg.longitude = oxts.packet.lon
        navsatfix_msg.altitude = oxts.packet.alt
        navsatfix_msg.status.service = 1
        bag.write(topic, navsatfix_msg, t=navsatfix_msg.header.stamp)


def save_gps_vel_data(bag, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = gps_frame_id
        twist_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        twist_msg.twist.linear.x = oxts.packet.vf
        twist_msg.twist.linear.y = oxts.packet.vl
        twist_msg.twist.linear.z = oxts.packet.vu
        twist_msg.twist.angular.x = oxts.packet.wf
        twist_msg.twist.angular.y = oxts.packet.wl
        twist_msg.twist.angular.z = oxts.packet.wu
        bag.write(topic, twist_msg, t=twist_msg.header.stamp)


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

    parser = argparse.ArgumentParser(description = "Convert KITTI dataset to ROS bag file the easy way!")
    parser.add_argument("-r", "--kitti_raw", help="KITTI raw dataset path", default="/home/dohoon/Datasets/kitti_raw/dataset")
    parser.add_argument("-p", "--path", help = "save path to the rosbag file.", default="./bag")
    parser.add_argument("-s", "--sequence", help="sequence number", default="07")
    args = parser.parse_args()

    date = seq_to_drive[args.sequence][:-11]
    drive = seq_to_drive[args.sequence][17:]
    kitti_raw = pykitti.raw(args.kitti_raw, date, drive)

    compression = rosbag.Compression.NONE

    save_path = os.path.join(args.path, "kitti_{}_synced".format(seq_to_drive[args.sequence]))
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    bag_fn = os.path.join(save_path, "kitti.bag")
    bag = rosbag.Bag(bag_fn, 'w', compression=compression)

    if not os.path.exists(kitti_raw.data_path):
        print('Path {} does not exists. Exiting.'.format(kitti_raw.data_path))
        sys.exit(1)

    if len(kitti_raw.timestamps) == 0:
        print('Dataset is empty? Exiting.')
        sys.exit(1)

    try:
        # IMU
        imu_frame_id = 'imu_link'
        imu_topic = '/kitti/oxts/imu'
        imu_raw_topic = '/imu_raw'
        gps_fix_topic = '/gps/fix'
        gps_vel_topic = '/gps/vel'
        velo_frame_id = 'velodyne'
        velo_topic = '/points_raw'

        T_base_link_to_imu = np.eye(4, 4)
        T_base_link_to_imu[0:3, 3] = [-2.71/2.0-0.05, 0.32, 0.93]

        util = pykitti.utils.read_calib_file(os.path.join(kitti_raw.calib_path, 'calib_cam_to_cam.txt'))

        # Export
        save_imu_data_raw(bag, kitti_raw, imu_frame_id, imu_raw_topic)
        save_gps_fix_data(bag, kitti_raw, imu_frame_id, gps_fix_topic)
        save_gps_vel_data(bag, kitti_raw, imu_frame_id, gps_vel_topic)
        save_velo_data(bag, kitti_raw, velo_frame_id, velo_topic)

    finally:
        print("## OVERVIEW ##")
        print(bag)
        bag.close()

