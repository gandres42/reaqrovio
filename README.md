# README #

# ReAqROVIO: Refractive Aquatic ROVIO #

This repository contains the implementation of the work **Online refractive camera model calibration in visual inertial odometry**. The method enables Visual Inertial Odometry (ROVIO) underwater without the need of camera calibration in the water. The cameras are only calirated in air and the refractive distortion due to water is rectified online by estimating the refractive index online in the state of the EKF of ROVIO.

Paper:
* https://doi.org/10.48550/arXiv.2409.12074 (IROS 2024)

## Key features:
1)  **Refractive camera model**: to enable online adaptation of the camera model to mitigate the refractive effects of underwater given the calibration in air.
2) **Barometric Depth Update**: Integration of barometric depth update.
3) **Multi-camera**: Extending unified refractive index estimation with odometry from multiple synchronized cameras.

## Multi-Camera Underwater Dataset
ReAqROIO can be seemlessly tested on our open-source [Multi Camera Underwater Visual Inertial Dataset](https://github.com/ntnu-arl/underwater-datasets?tab=readme-ov-file#subset-3-trajectories-with-ground-truth-from-motion-capture). We use the 3rd subset of the dataset for our experiments.

## Video
[YouTube](https://www.youtube.com/watch?v=i9Cz8xE-0RI&ab_channel=KostasAlexis)

![alt text](media/Traj1.gif)

## Usage instructions
### Rosbag download and play

1. Download rosbags for trajectory (eg. Traj 1 from the dataset [link](https://ntnu.app.box.com/s/8tpgvtqlrhol8rts929x9h3rbo57mat6))
2. Move all the bag rosbag files to a directory (eg. ```<path to dataset>/Traj1```)
```
# Mono: (i.e setting ROVIO_NCAM to 1 in CMakeList):
 
rosbag play <path to dataset>/Traj1/* --topics /alphasense_driver_ros/cam0 /alphasense_driver_ros/imu --clock

# Stereo: (i.e setting ROVIO_NCAM to 2 in CMakeList):

rosbag play <path to dataset>/Traj1/* --topics /alphasense_driver_ros/cam0 /alphasense_driver_ros/cam1 /alphasense_driver_ros/imu --clock
```

### Rovio parameters
#### CMakeList
The CMakeList controls the maximum number of features used in the state of rovio, the number of cameras uses and multi-level patch parameters. 
** Parameters**
```
set(ROVIO_NMAXFEATURE 25 CACHE STRING "Number of features for ROVIO")
set(ROVIO_NCAM 2 CACHE STRING "Number of enabled cameras")
```
#### Launch file
```
refractive_index = 1.34
```
#### Info file 
```
refractiveCalibration true 
useObservabilityCheck true
```

## Potential issues and fixes
Incase ```git submodule update fails``` please clone ```git@github.com:Mohit505Git/lightweight_filtering.git``` in the lightweight_filter directory.


# Updated ROVIO Readme for ReAq ROVIO


Papers:
* http://dx.doi.org/10.3929/ethz-a-010566547 (IROS 2015)
* http://dx.doi.org/10.1177/0278364917728574 (IJRR 2017)

Please also have a look at the wiki: https://github.com/ethz-asl/rovio/wiki

### Install without opengl scene ###
Dependencies:
* ros
* kindr (https://github.com/ethz-asl/kindr)
* lightweight_filtering (as submodule, use "git submodule update --init --recursive")

```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Install with opengl scene ###
Additional dependencies: opengl, glut, glew (sudo apt-get install freeglut3-dev, sudo apt-get install libglew-dev)
```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
```

### Euroc Datasets ###
The rovio_node.launch file loads parameters such that ROVIO runs properly on the Euroc datasets. The datasets are available under:
http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

### Further notes ###
* Camera matrix and distortion parameters should be provided by a yaml file or loaded through rosparam
* The cfg/rovio.info provides most parameters for rovio. The camera extrinsics qCM (quaternion from IMU to camera frame, Hamilton-convention) and MrMC (Translation between IMU and Camera expressed in the IMU frame) should also be set there. They are being estimated during runtime so only a rough guess should be sufficient.
* Especially for application with little motion fixing the IMU-camera extrinsics can be beneficial. This can be done by setting the parameter doVECalibration to false. Please be carefull that the overall robustness and accuracy can be very sensitive to bad extrinsic calibrations.
