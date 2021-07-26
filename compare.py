import argparse
import time
import yaml
from subprocess import *
from rosbag.bag import Bag
import matplotlib.pyplot as plt

import src.trajectory as tj
import src.error as error

slam_possible = {'aloam': 'record_aloam.launch', 
                 'lego_loam': 'record_lego_loam.launch',
                 'lio_sam': 'record_lio_sam.launch',}

parser = argparse.ArgumentParser(description="LiDAR SLAM Benchmark")
parser.add_argument('--slam', nargs="+", dest='slam_lists', help='SLAM algorithms that want to compare: aloam, lego_loam, lio_sam')
parser.add_argument('--bagfile', dest='bagfile_name', help='Name of KITTI dataset bagfile in "slam_benchmark/PathRecorder/bag" directory')
parser.add_argument('--plot', dest='plot_arg', choices=['all', 'traj', 'error', 'stat'], default='all', help='Plot options: all, traj, error, stat')
parser.add_argument('--no_play', dest='play_flag', action='store_false', help='If you already have recorded result bag file, add --no_play')
args = parser.parse_args()

if args.slam_lists is None:
	print("No specified SLAM Lists. Run default algorithms: ALOAM, LEGO_LOAM, LIO_SAM")
	args.slam_lists = ["aloam", "lego_loam", "lio_sam"]

if 'all' in args.slam_lists:
    args.slam_lists = ["aloam", "lego_loam", "lio_sam"]

for slam in args.slam_lists:
    if slam not in slam_possible:
        raise ValueError("%s algorithm is not available!" % slam)

if args.bagfile_name is None:
	raise parser.error("Please set bag file with --bagfile parser")

class CompareSLAM():
    def __init__(self, slam_lists, bagfile_name, plot_arg):
        self.slam_lists = slam_lists
        self.bagfile_name = bagfile_name
        self.bag_dir = 'PathRecorder/bag/' + args.bagfile_name
        bag_info_dict = yaml.load(Bag(self.bag_dir + '.bag', 'r')._get_yaml_info())
        self.bag_duration = bag_info_dict['duration']
        self.plot_arg = plot_arg
        self.file_list = []
        self.file_list.append(self.bag_dir + '/kitti_gt.bag')
        for slam in self.slam_lists:
            self.file_list.append(self.bag_dir +'/'+ slam + '_path.bag')
        
    def play_algorithm(self):     
        for slam in self.slam_lists:            
            print("%s algorithm is running ..." % slam)
            print("bag file duration: %i" % self.bag_duration)
            print(self.bagfile_name)
            p1 = Popen(["roslaunch", "path_recorder", slam_possible[slam], "kitti_bag:=" + self.bagfile_name])
            
            start_time = time.time()
            currnet_time = start_time
            while True:
                time.sleep(1)
                current_time = time.time()
                if current_time - start_time >= self.bag_duration:
                    print("finishing %s ..."% slam)
                    p1.terminate()
                    p1.wait()
                    break
            
        print("Finished all algorithm")
        self.plot()

    def plot(self):
        plot_arg = self.plot_arg
        file_list = self.file_list
        gt, tj_list = self.traj_process(file_list)
        if plot_arg == 'traj' : return self.plot_traj(gt, tj_list)
        else:
            error_list = self.error_process(gt, tj_list)
            return self.plot_error(plot_arg, gt, tj_list, error_list)

    def traj_process(self, data_files):
        tj_list = []
        gt = None
        for file in data_files:
            if (file.endswith('.bag') or file.endswith('.txt')):
                trajectory = tj.Trajectory(file)
                if(not trajectory.is_gt):
                    tj_list.append(trajectory)
                else: gt = trajectory
            else: print("Unsupported .{} file type".format(file.split('.')[-1]))
        return gt, tj_list

    def error_process(self, gt, tj_list):
        error_list = []
        if(gt == None):
            print('Need ground truth for error calculation.')
            return
        for tj in tj_list:
            error_list.append(error.Error(gt, tj))
        return error_list

    def plot_traj(self, gt, tj_list):
        print("plotting...")
        tj.plotXYZ(gt, tj_list)
        tj.plot2D('xy', gt, tj_list)
        tj.plot3D(gt, tj_list)
        return plt.show()

    def plot_error(self, plot_arg, gt, tj_list, error_list):
        print("plotting...")
        if (plot_arg == 'all'):
            tj.plotXYZ(gt, tj_list)
            tj.plot2D('xy', gt, tj_list)
            tj.plot3D(gt, tj_list)
        
            error.plotAPE(error_list)
            error.plotAPEStats(error_list)
            error.plotRPE(error_list)
            error.plotRPEStats(error_list)
            
        if (plot_arg == 'error'):    
            error.plotAPE(error_list)
            error.plotAPEStats(error_list)
            error.plotRPE(error_list)
            error.plotRPEStats(error_list)
            
        if (plot_arg == 'stat'):
            error.plotAPEStats(error_list)
            error.plotRPEStats(error_list)
        return self.print_result(error_list), plt.show()
    
    def print_result(self, error_list):
        for error in error_list:
            print(error.name)
            print("APE[m]   : {}".format(error.ape_tans_stat[5]))
            print("RPE[m]   : {}".format(error.rpe_tans_stat[5]))
            print("RPE[rad] : {}".format(error.rpe_rot_stat[0]))


if __name__ == '__main__':
    benchmark = CompareSLAM(args.slam_lists, args.bagfile_name, args.plot_arg)
    if args.play_flag:
        benchmark.play_algorithm()
    else:
        benchmark.plot()
            