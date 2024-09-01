# EE-VINS: An Event-Enhanced Visual-Inertial SLAM Scheme for Dynamic Environments

## Test Env.

This code is tested on

* Linux 20.04 LTS
* ROS Noetic
* OpenCV 4.2.0

## :package: Prerequisites

The dependency of EE-VINS is equal to that of VINS-Fusion.

### 1. **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04, 18.04 or 20.04.
ROS Kinetic, Melodic or Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 3. **Support file from VINS-Fusion**

Due to the limiting file size of Github, we need **one** package and **two** files from the [VINS-Fusion repository](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/tree/master/support_files).

1. Set the `camera_models` package in your workspace, which is included in VINS-Fusion.
2. Copy `support_files/brief_k10L6.bin` in VINS-Fusion into our `support_files` folder.
2. Copy `support_files/brief_pattern.yml` in VINS-Fusion into our `support_files` folder.

### 4. **Other support files**

Due to the limiting file size of Github, we need **two** packages from the [catkin_simple repository](https://github.com/catkin/catkin_simple) and the [rpg_dvs_ros repository](https://github.com/uzh-rpg/rpg_dvs_ros), respectively.

1. Set the `catkin_simple` package in your workspace.
2. Set the `rpg_dvs_ros` package in your workspace. 

## :building_construction: How to build

> Please follow the below commands to build EE-VINS (on ROS).

``` bash
$ cd ~/EE-VINS/src 
$ git clone https://github.com/huangfeng95/EE-VINS
$ cd ../
$ catkin_make  
(or if you use catkin tools) catkin build
$ source ~/EE-VINS/devel/setup.bash
```

## :runner: To run the demo codes

### VIODE dataset examples

You can download the dataset from : https://github.com/kminoda/VIODE

Note that the larger the number of bag files in the VIODE dataset is, the more dynamic objects exist.

#### 1. **VIODE sequence with monocular camera + IMU**

``` bash
$ roslaunch EE-VINS viode_mono.launch
$ rosbag play 3_high.bag (or 0_none.bag, 1_low.bag, ...)
```


### ECMD dataset examples

You can download the dataset from : https://arclab-hku.github.io/ecmd/

``` bash
$ roslaunch EE-VINS ECMD_mono.launch
$ rosbag play Dense_street_day_easy_a.bag.bag (or Dense_street_day_easy_b.bag, ...)
```

## :bookmark: Acknowledgements

The authors of: [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

The authors of: [dynaVINS](https://github.com/url-kaist/dynaVINS)

The authors of: [VIODE](https://github.com/kminoda/VIODE)

The authors of: [ECMD](https://arclab-hku.github.io/ecmd/)

The authors of: [evo](https://github.com/MichaelGrupp/evo)
