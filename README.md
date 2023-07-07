# 功能介绍

hobot_vio package 是地平线在Ros2开发的视觉惯性里程计（Visual Inertial Odometry, VIO）算法，
用于机器人室内外的定位。本文以realsenseD435i为作为测试传感器，使用本算法前，请先安装realsense的ROS2 package。

# 编译

## 依赖库

### x3 依赖

- pthread
- dl
- OpenCv
- Eigen

ros package：

- rclcpp
- sensor_msgs
- nav_msgs
- cv_bridge
- tf
- tf2_ros

## 开发环境

- 编程语言: C/C++
- 开发平台: X3
- 系统版本：Ubuntu 20.04
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### 编译选项


### Ubuntu板端编译X3版本

1. 编译环境确认
   - 板端已安装X3 Ubuntu系统。
   - 当前编译终端已设置TogetheROS环境变量：`source PATH/setup.bash`。其中PATH为TogetheROS的安装路径。
   - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`

2. 编译
 编译命令：`colcon build --packages-select hobot_vio --cmake-args`


### Docker交叉编译X3版本

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetheROS。docker安装、交叉编译说明、TogetheROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

```shell
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select hobot_vio \
   --merge-install \
   --cmake-force-configure \
   --cmake-args \
   --no-warn-unused-cli \
   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
```

## 注意事项

# 使用介绍

## 输入参数

| 参数名 | 类型 | 解释  | 是否必须 | 默认值 |
| ----- | ----| -----| ------- | -----|
| path_config  | std::string | vio算法配置文件路径 | 是        | /opt/tros/lib/hobot_vio/config/realsenseD435i.yaml |
| image_topic  | std::string | vio算法订阅的图像数据话题名  | 是 | /camera/infra1/image_rect_raw |
| imu_topic    | std::string | vio算法订阅的IMU数据话题名  | 是 | /camera/imu  |
| sample_gap  | std::string | vio算法处理频率，1表示每帧图像都会参与轨迹计算，2表示每两帧图像计算一次，依此类推 | 是  | 2 |

## 输出topic

| topic名 | 类型 | 解释  |
| ----- | ----| -----| 
| horizon_vio/horizon_vio_path  | nav_msgs::msg::Path | vio算法输出的机器人运动轨迹  |

## 运行

编译成功后，将生成的install路径拷贝到地平线旭日X3开发板上（如果是在X3上编译，忽略拷贝步骤），
并执行如下命令运行：


### **Ubuntu X3**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# 启动vio节点
ros2 run hobot_vio hobot_vio

```

### **Ubuntu X3 Launch启动**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# 启动agent node
ros2 launch hobot_vio hobot_vio.launch.py
```

### **Linux X3**

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# 启动 vio node
./install/lib/hobot_vio/hobot_vio
```

# 结果分析

## X3 日志信息

```bash

root@ubuntu:/mnt/nfs20/zhy/c-lab/tros_ws/install# ros2 launch hobot_vio hobot_vio.launch.py
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-07-19-48-31-464088-ubuntu-562910
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_vio-1]: process started with pid [563077]
[INFO] [ros2 launch realsense2_camera rs_launch.py  depth_module.profile:=640x480x30 enable_depth:=false enable_color:=false enable_gyro:=true enable_accel:=true enable_sync:=true gyro_fps:=200 accel_fps:=200 unite_imu_method:=2 enable_infra1:=true-2]: process started with pid [563081]
[hobot_vio-1] T_CtoI:
[hobot_vio-1]    0.999934   0.0103587   0.0049969   0.0270761
[hobot_vio-1]  -0.0104067    0.999899  0.00967935 -0.00272628
[hobot_vio-1] -0.00489613 -0.00973072    0.999941  -0.0518149
[hobot_vio-1]           0           0           0           1
[hobot_vio-1] system use_rtk_: 0
[hobot_vio-1] [static initializer] not enough imu readings
[hobot_vio-1] [static initializer] not enough imu readings
[hobot_vio-1] [static initializer] IMU belows th 0.011508, 0.00274453 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.0105996, 0.00273085 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00964632, 0.00280866 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00892132, 0.00279346 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00816016, 0.00281761 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00776753, 0.00277049 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00744219, 0.00274874 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.420251, 0.36058 < 0.5, 0
[hobot_vio-1] HorizonVIO Successfully initialized!
[hobot_vio-1] [WARN] [1688730518.534178615] [horizon_vio_node]: Localization position[x, y, z]: [0.0225533, -0.0504654, 0.00943574]
[hobot_vio-1] [WARN] [1688730518.534634139] [horizon_vio_node]: Image time 1688730518.314490318
[hobot_vio-1] [WARN] [1688730518.621440869] [horizon_vio_node]: Localization position[x, y, z]: [0.0231779, -0.0533648, 0.00787081]
[hobot_vio-1] [WARN] [1688730518.621558739] [horizon_vio_node]: Image time 1688730518.380982161
[hobot_vio-1] [WARN] [1688730518.743525086] [horizon_vio_node]: Localization position[x, y, z]: [0.0290396, -0.0610474, 0.0106718]
[hobot_vio-1] [WARN] [1688730518.743637249] [horizon_vio_node]: Image time 1688730518.447472572
[hobot_vio-1] [WARN] [1688730518.866076119] [horizon_vio_node]: Localization position[x, y, z]: [0.0381324, -0.0737757, 0.0164843]
[hobot_vio-1] [WARN] [1688730518.866186156] [horizon_vio_node]: Image time 1688730518.513962030
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 338
[hobot_vio-1] 132.853 ms all consumed
[hobot_vio-1] travel(m): 0.000
[hobot_vio-1] [WARN] [1688730519.002002975] [horizon_vio_node]: Localization position[x, y, z]: [0.05018, -0.088422, 0.0240244]
[hobot_vio-1] [WARN] [1688730519.002130095] [horizon_vio_node]: Image time 1688730518.580449104
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 31
[hobot_vio-1] 142.996 ms all consumed
[hobot_vio-1] travel(m): 0.014
[hobot_vio-1] [WARN] [1688730519.146149433] [horizon_vio_node]: Localization position[x, y, z]: [0.0167176, -0.0189649, 0.0588413]
[hobot_vio-1] [WARN] [1688730519.146279428] [horizon_vio_node]: Image time 1688730518.646935701
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 26
[hobot_vio-1] 96.911 ms all consumed
[hobot_vio-1] travel(m): 0.025
[hobot_vio-1] [WARN] [1688730519.244168068] [horizon_vio_node]: Localization position[x, y, z]: [0.000805884, 0.0134815, 0.0730707]
[hobot_vio-1] [WARN] [1688730519.244270439] [horizon_vio_node]: Image time 1688730518.713421583
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 23
[hobot_vio-1] 52.470 ms all consumed
[hobot_vio-1] travel(m): 0.034
[hobot_vio-1] [WARN] [1688730519.297642444] [horizon_vio_node]: Localization position[x, y, z]: [0.00226324, 0.0120054, 0.0796328]
[hobot_vio-1] [WARN] [1688730519.297738190] [horizon_vio_node]: Image time 1688730518.779906034
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 33
[hobot_vio-1] 47.407 ms all consumed
[hobot_vio-1] travel(m): 0.042

```