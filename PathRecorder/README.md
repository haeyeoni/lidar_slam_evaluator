PathRecorder
===

This module is for recording a `nav_msgs::Path` formatted trajectory that is generated by Lidar SLAM nodes. \
A-LOAM, LeGO-LOAM and LIO-SAM are tested with an input data bagfiles converted by the modified `kitti2bag`. 
   
   

Try some launch files in `PathRecorder/launch/`. 

**Example:**

```bash
roslaunch path_recorder record_aloam.launch kitti_bag:=kitti_2011_09_30_drive_0027_synced
```

