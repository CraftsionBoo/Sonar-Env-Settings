# DAVE Sonar Simulation Environment Setup

> [中文文档](README_CN.md)

## Project Overview

DAVE (Dave Virtual Environment) is a simulation environment designed to support rapid testing and evaluation of underwater robotic solutions, particularly for autonomous underwater vehicles (AUVs, UUVs) performing autonomous operations and completing autonomous driving tasks. The environment is built on existing ROS and Gazebo architectures, specifically the UUV Simulator and WHOI's DS_SIM. This documentation primarily records the setup of its sonar system for testing on Jetson devices.

### Dependencies
- ROS Noetic
- OpenCV 4.4.0

## Installation Guide

### 1. Clone the Repository
```bash
mkdir -p ~/uuv_ws/src
cd ~/uuv_ws/src
git clone https://github.com/Field-Robotics-Lab/dave.git
```

### 2. Modify Configuration and Clone Dependencies
- Edit `dave/extras/repos/dave_sim.repos` to remove dockwater (not required)
- Install vcs tool:
    ```bash
    sudo pip3 install -U vcstool
    cd ~/uuv_ws/src
    vcs import --skip-existing --input dave/extras/repos/dave_sim.repos
    ```
> **Note**: If you encounter "Permission denied (publickey)" error, follow the GitHub SSH setup guide:
> [GitHub SSH Troubleshooting](https://docs.github.com/en/authentication/troubleshooting-ssh/error-permission-denied-publickey)

**Alternative Method - Manual Cloning** (Stable, Recommended):
```bash
cd ~/uuv_ws/src
git clone -b nps_dev https://github.com/Field-Robotics-Lab/ds_msgs.git
git clone -b nps_dev https://github.com/Field-Robotics-Lab/ds_sim.git
git clone -b master https://github.com/uuvsimulator/eca_a9.git
git clone -b master https://github.com/uuvsimulator/rexrov2.git
git clone -b master https://github.com/field-robotics-lab/uuv_manipulators.git
git clone -b master https://github.com/field-robotics-lab/uuv_simulator.git
git clone -b main https://github.com/field-robotics-lab/nps_uw_multibeam_sonar.git
git clone -b tags/v2.0.0 https://github.com/apl-ocean-engineering/marine_msgs.git
```

### 3. Build DAVE
```bash
# Install build tool catkin_tools
pip3 install -U catkin_tools
cd ~/uuv_ws
catkin build
```

> **Note**: If build fails, try building projects individually to identify problematic libraries:
```bash
catkin build ${project_name}
```

### 4. Run Simulation and Results
```bash
# Standalone Ray Simulation
source devel/setup.bash
roslaunch nps_uw_multibeam_sonar local_search_blueview_p900_nps_multibeam_urdf_standalone_ray.launch
```
![Standalone Ray Simulation](assets/standalone_ray.png)

```bash
# Ray Simulation
source devel/setup.bash
roslaunch nps_uw_multibeam_sonar local_search_blueview_p900_nps_multibeam_ray.launch
```
![Ray Simulation](assets/ray.png)

## Troubleshooting

### Q1: Missing libgazebo_ros_velodyne_gpu_laser.so Depen
```bash
# This library file is not included in the program. You can:
# 1. Search for libgazebo_ros_velodyne_gpu_laser.so in existing docker version
sudo find / -name "libgazebo_ros_velodyne_gpu_laser.so"
# 2. Check the libs folder
```

### Q2: ROS Vision Libraries (cv_bridge and image_geometry) Errors Due to OpenCV Version
```bash
# Clone and rebuild ROS vision libraries
git clone https://github.com/ros-perception/vision_opencv.git

# Same process for both cv_bridge and image_geometry
cd src/cv_bridge # or cd src/image_geometry
# Modify CMakeLists.txt to specify OpenCV version: find_package(OpenCV 4.4.0 REQUIRED)
mkdir build && cd build 
cmake —DCMKAE_INSTALL_PREFIX=/opt/ros/noetic ..
make -j8
sudo make install 
```

## References
- [DAVE GitHub Repository](https://github.com/Field-Robotics-Lab/DAVE)
- [DAVE Documentation](https://field-robotics-lab.github.io/dave.doc/)
