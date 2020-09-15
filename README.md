# 3D Radiation Mapper with Voxblox
This branch is used to map radiation intensities into a 3D environmental representation. This is used e.g. for [ENRICH](https://enrich.european-robotics.eu).

![rnm_example_gif](readme_files/rnm_mesh.gif)

## Requirements
Before you start, make sure that you have selected the right branches.

| Repository | Branch |
|:-:|:-:|
| **voxblox**<br>(this repository) | [`radio_nuclear_mapper`](https://git.sim.informatik.tu-darmstadt.de/hector/hector_voxblox/-/tree/radio_nuclear_mapper)<br>(this branch) |
| **radiological_nuclear_mapper** | [`voxblox`](https://git.sim.informatik.tu-darmstadt.de/hector/hector_enrich/-/tree/voxblox/radiological_nuclear_mapper) |
| **hector_vehicle_launch** | [`radiological_nuclear_mapper`](https://github.com/tu-darmstadt-ros-pkg/hector_vehicle_launch/tree/radiological_nuclear_mapper) |

## How To Run 3D Radiation Mapper?
**TL/DR:** [Use this](https://git.sim.informatik.tu-darmstadt.de/hector/hector_voxblox/-/blob/radio_nuclear_mapper/README.md#alternative-use-the-super-cool-all-in-one-terminator-start-script)

Decide for one ENRICH run or create new file(s) for a new run. Make sure you have the corresponding bag files. If you have all required files collected continue with the instructions for manual or automated start.

### Launch Files

#### Choose the Launch File for Voxblox
The launch files for Voxblox are located in branch **radio_nuclear_mapper** of repository **hector_vehicle_launch**:

- [`hector_sensor_proc_launch/launch/voxblox_rnm_enrich_2017.launch`](https://github.com/tu-darmstadt-ros-pkg/hector_vehicle_launch/blob/radiological_nuclear_mapper/hector_sensor_proc_launch/launch/voxblox_rnm_enrich_2017.launch) (for bag files of ENRICH 2017)
- [`hector_sensor_proc_launch/launch/voxblox_rnm_enrich_2019_1.launch`](https://github.com/tu-darmstadt-ros-pkg/hector_vehicle_launch/blob/radiological_nuclear_mapper/hector_sensor_proc_launch/launch/voxblox_rnm_enrich_2019_1.launch) (for bag files of ENRICH 2019 Run 1)*
- [`hector_sensor_proc_launch/launch/voxblox_rnm_enrich_2019_2.launch`](https://github.com/tu-darmstadt-ros-pkg/hector_vehicle_launch/blob/radiological_nuclear_mapper/hector_sensor_proc_launch/launch/voxblox_rnm_enrich_2019_2.launch) (for bag files of ENRICH 2019 Run 2)

\* The mesh shown above was created with this data.

#### Choose the Launch File for Static TF Publisher
This step is **only** required if only a single `static_tf` message in the bag files is published once at the beginning. In this case skipping the first time of a recording will effect, that static transformations are unknown. For ENRICH 2017 thats the case. Therefore do following:

1. [Generate a Static Transformation Publisher](https://git.sim.informatik.tu-darmstadt.de/hector/hector_enrich/-/blob/voxblox/generate_static_transformation_publisher/README.md) or choose [static_tf_2017.launch](https://git.sim.informatik.tu-darmstadt.de/hector/hector_enrich/-/blob/voxblox/generate_static_transformation_publisher/static_tf_2017.launch)
2. Copy the launch file to directory `~/hector/src/robot_launch/robot_postproc_launch/launch`

#### Is there a LIDAR pointcloud self filter?
- ENRICH 2017: No
- ENRICH 2019: Yes ([vlp16_self_filter.launch](https://git.sim.informatik.tu-darmstadt.de/drz/drz_telemax_launch/-/blob/master/drz_telemax_onboard_launch/launch/lidar_proc/vlp16_self_filter.launch))

Keep the launch file in mind for later.

#### Pro Tip: Use predefined RVIZ configurations
Also in branch **radio_nuclear_mapper** of repository **hector_vehicle_launch** are a few prepared .rviz files these describe the configuration of [RVIZ](http://wiki.ros.org/rviz) windows:

- [`hector_sensor_proc_launch/enrich2017_mesh.rviz`](https://github.com/tu-darmstadt-ros-pkg/hector_vehicle_launch/blob/radiological_nuclear_mapper/hector_sensor_proc_launch/enrich2017_mesh.rviz) (for ENRICH 2017 run)
- [`hector_sensor_proc_launch/enrich2019_mesh.rviz`](https://github.com/tu-darmstadt-ros-pkg/hector_vehicle_launch/blob/radiological_nuclear_mapper/hector_sensor_proc_launch/enrich2019_mesh.rviz) (for both ENRICH 2019 runs)

### Start Manually

Ensure `roscore` is running. Then you need six additional terminal windows. Execute these commands one after the other in each window: 

1. `roslaunch robot_postproc_launch play_with_recorded_tf.launc`
2. (Only if a static tf publisher is required)`roslaunch robot_postproc_launch static_tf_XXXX.launch`
3. (Only if a pointcloud filter exists) `roslaunch drz_telemax_onboard_launch vlp16_self_filter.launch`
3. `roslaunch drz_telemax_onboard_launch vlp16_self_filter.launch`
4. `roslaunch hector_sensor_proc_launch voxblox_rnm_enrich_2017.launch`
5. `rviz -d ~/hector/src/hector_vehicle_launch/hector_sensor_proc_launch/enrich2019_mesh.rviz`
6. `rosparam set use_sim_time true && cd /media/psf/Home/enrich/2019/competition_run_1/bags && rosbag play *.bag --clock -r 1 -s 220 /spin_laser/vlp16:=/spin_laser/vlp16_trash`

#### Examples

### ALTERNATIVE: Use the Super Cool All In One Terminator Start Script

Ensure `roscore` is running.

# Voxblox

![voxblox_small](https://cloud.githubusercontent.com/assets/5616392/15180357/536a8776-1781-11e6-8c1d-f2dfa34b1408.gif)

Voxblox is a volumetric mapping library based mainly on Truncated Signed Distance Fields (TSDFs). It varies from other SDF libraries in the following ways:
 * CPU-only, can be run single-threaded or multi-threaded for some integrators
 * Support for multiple different layer types (containing different types of voxels)
 * Serialization using protobufs
 * Different ways of handling weighting during merging
 * Different ways of inserting pose information about scans
 * Tight ROS integration (in voxblox_ros package)
 * Easily extensible with whatever integrators you want
 * Features an implementation of building Euclidean Signed Distance Fields (ESDFs, EDTs) directly from TSDFs.

**If you're looking for skeletonization/sparse topology or planning applications, please refer to the [mav_voxblox_planning](https://github.com/ethz-asl/mav_voxblox_planning) repo.**
**If you want to create ground truth maps from meshes or gazebo environments, please check out the [voxblox_ground_truth](https://github.com/ethz-asl/voxblox_ground_truth) pakage!**

![example_gif](http://i.imgur.com/2wLztFm.gif)

# Documentation
* All voxblox documentation can be found on [our readthedocs page](https://voxblox.readthedocs.io/en/latest/index.html)

## Table of Contents
* [Paper and Video](#paper-and-video)
* [Credits](#credits)
* [Example Outputs](https://voxblox.readthedocs.io/en/latest/pages/Example-Outputs.html)
* [Performance](https://voxblox.readthedocs.io/en/latest/pages/Performance.html)
* [Installation](https://voxblox.readthedocs.io/en/latest/pages/Installation.html)
* [Running Voxblox](https://voxblox.readthedocs.io/en/latest/pages/Running-Voxblox.html)
* [Using Voxblox for Planning](https://voxblox.readthedocs.io/en/latest/pages/Using-Voxblox-for-Planning.html)
* [Transformations in Voxblox](https://voxblox.readthedocs.io/en/latest/pages/Transformations.html)
* [Contributing to Voxblox](https://voxblox.readthedocs.io/en/latest/pages/Modifying-and-Contributing.html)
* [Library API](https://voxblox.readthedocs.io/en/latest/api/library_root.html)

# Paper and Video
A video showing sample output from voxblox can be seen [here](https://www.youtube.com/watch?v=PlqT5zNsvwM).
A video of voxblox being used for online planning on-board a multicopter can be seen [here](https://youtu.be/lrGSwAPzMOQ).

If using voxblox for scientific publications, please cite the following paper, available [here](http://helenol.github.io/publications/iros_2017_voxblox.pdf):

Helen Oleynikova, Zachary Taylor, Marius Fehr, Juan Nieto, and Roland Siegwart, “**Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning**”, in *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2017.

```latex
@inproceedings{oleynikova2017voxblox,
  author={Oleynikova, Helen and Taylor, Zachary and Fehr, Marius and Siegwart, Roland and  Nieto, Juan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning},
  year={2017}
}
```

# Credits
This library was written primarily by Helen Oleynikova and Marius Fehr, with significant contributions from Zachary Taylor, Alexander Millane, and others. The marching cubes meshing and ROS mesh generation were taken or heavily derived from [open_chisel](https://github.com/personalrobotics/OpenChisel). We've retained the copyright headers for the relevant files.

![offline_manifold](https://i.imgur.com/pvHhVsL.png)
