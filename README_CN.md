# DAVE 声纳仿真环境搭建

>[English Version](README.md)

## 项目概述

Dave虚拟环境（DAVE）是一个模拟环境，旨在支持水下机器人解决方案的快速测试和评估，尤其是水下车辆（AUVS，UUV）自动操作并完成涉及自动驾驶的任务。环境建立在现有的ROS和Gazebo架构上，尤其是UUV模拟器和WHOI的DS_SIM。本次主要是记录搭建其声纳系统于jetson设备上进行测试

### 依赖
- ROS Noetic
- OpenCV 4.4.0

## 安装指南

### 1. 克隆仓库
```bash
mkdir -p ~/uuv_ws/src
cd ~/uuv_ws/src
git clone https://github.com/Field-Robotics-Lab/dave.git
```

### 2. 修改配置并克隆依赖
- 编辑 `dave/extras/repos/dave_sim.repos` 删除 dockwater（不需要）
- 安装 vcs 工具：
```bash
sudo pip3 install -U vcstool
cd ~/uuv_ws/src
vcs import --skip-existing --input dave/extras/repos/dave_sim.repos
```
> **注意**：如果遇到"Permission denied (publickey)"错误，请按照 GitHub SSH 设置指南操作：
> [GitHub SSH 故障排除](https://docs.github.com/en/authentication/troubleshooting-ssh/error-permission-denied-publickey)

**替代方式--手动克隆方式**(稳定，建议选择)：
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

### 3. 编译 DAVE
```bash
# 安装构建工具 catkin_tools
pip3 install -U catkin_tools
cd ~/uuv_ws
catkin build
```
> **注意**：如果构建失败，可以尝试单独构建项目，便于寻找出错库
```bash
catkin build ${project_name}
```

### 4. 运行仿真以及结果
```bash
# 独立射线仿真
source devel/setup.bash
roslaunch nps_uw_multibeam_sonar local_search_blueview_p900_nps_multibeam_urdf_standalone_ray.launch
```
![独立射线仿真](assets/standalone_ray.png)

```bash
# 射线仿真
source devel/setup.bash
roslaunch nps_uw_multibeam_sonar local_search_blueview_p900_nps_multibeam_ray.launch
```
![射线仿真](assets/ray.png)

## 故障排除

### Q1: 缺少 libgazebo_ros_velodyne_gpu_laser.so 依赖库
```bash
# 这个库文件不在程序中，可以选择：
# 1. 从已有docker版本中寻找libgazebo_ros_velodyne_gpu_laser.so
sudo find / -name "libgazebo_ros_velodyne_gpu_laser.so"
# 2. 见libs文件夹
```

### Q2: 由于opencv版本问题导致ROS视觉库cv_bridge和image_geomotry报错
```bash
# 克隆并重新构建ROS视觉库
git clone https://github.com/ros-perception/vision_opencv.git

# cv_bridge和image_geometry方式一样
cd src/cv_bridge # cd src/image_geometry
# 修改 CMakeLists.txt 指定 OpenCV 版本：find_package(OpenCV 4.4.0 REQUIRED)
mkdir build && cd build 
cmake —DCMKAE_INSTALL_PREFIX=/opt/ros/noetic ..
make -j8
sudo make install 
```

## 参考资料
- [DAVE GitHub 仓库](https://github.com/Field-Robotics-Lab/DAVE)
- [DAVE 文档](https://field-robotics-lab.github.io/dave.doc/) 