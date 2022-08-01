# ESKF

a fusion method of GPS and IMU based on error state kalman filter

## Prerequisites

1. ROS melodic

- package: nmea_navsat_driver

## Datasets

1. EU_Longterm

- download from [utbm_robocar_dataset_20190131_noimage.bag](https://drive.utbm.fr/s/H4fH99RH8YwywY3)

## Build ESKF on ROS

```
cd ~/catkin_ws/src
git clone https://github.com/cquxyw/ESKF.git
cd ../
catkin_make
```

## Test on public dataset

1. run utbm_robocar_dataset_20190131_noimage.bag

```
rosbag play [dataset_dir]/utbm_robocar_dataset_20190131_noimage.bag
```

2. launch eskf package
open new terminal:
```
source ~/catkin_ws/devel/setup.bash
roslaunch eskf eskf.launch
```
