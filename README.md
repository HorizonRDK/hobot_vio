# 功能介绍

视觉惯性里程计（Visual Inertial Odometry，VIO）是融合相机与惯性测量单元（Inertial Measurement Unit，IMU）数据实现机器人定位的算法。VIO定位算法具有成本低、适用环境广等优点，在室外环境下能够有效弥补卫星定位中遮挡、多路径干扰等失效场景。优秀、鲁棒的VIO算法是实现室外高精度导航定位的关键。
![](./imgs/hobot_vio_rviz.jpeg)

代码仓库：<https://github.com/HorizonRDK/hobot_vio.git>

# 物料清单

| 机器人名称          | 生产厂家 | 参考链接                                                     |
| :------------------ | -------- | ------------------------------------------------------------ |
| RDK X3             | 多厂家 | [点击跳转](https://developer.horizon.cc/sunrise) |
| realsense             | Intel RealSense D435i |                |

# 使用方法

## 准备工作

在体验之前，需要具备以下基本条件：

- 地平线RDK已烧录好地平线提供的Ubuntu 20.04系统镜像
- realsense确连接到RDK X3 USB 3.0接口

算法订阅realsense相机的图像和IMU数据作为算法的输入，经过计算得到相机的轨迹信息，并通过ROS2的话题机制发布相机的运动轨迹，轨迹结果可在PC的rviz2软件查看。算法的输入和输出topic如下表所示：



# 使用方法

**1.安装功能包**

启动机器人后，通过终端或者VNC连接机器人，点击本页面右上方的“一键部署”按钮，复制如下命令在RDK的系统上运行，完成相关Node的安装。

```bash
sudo apt update
sudo apt install -y tros-vio
```


**2.运行VIO功能**

启动命令：

```shell
# 配置tros.b环境
source /opt/ros/foxy/setup.bash
source /opt/tros/local_setup.bash

ros2 launch hobot_vio hobot_vio.launch.py 
```

**3.查看效果**




# 接口说明

## 输入topic
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

# 参考资料


# 常见问题

