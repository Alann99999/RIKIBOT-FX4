自用 RIKIBOT-FX4 小车的工作空间，可实现基于 Slam_gmapping 的建图（使用 RPLIDAR-C1 单线激光雷达）、move_base 的路径规划导航、explore_lite 的自主探索。
以下为操作指南：

# Rikibot-FX4 小车键盘控制指南

## 概述
本指南描述如何通过ROS (Robot Operating System) 使用键盘远程控制Rikibot-FX4小车。适用于rikibot@robot环境，支持LIDAR (rplidar) 和摄像头 (Intel)。所有操作在终端中执行。

## 前提条件
- **硬件**：小车电源开启，USB连接正常（串口 `/dev/rikibase` 可用）。
- **软件**：ROS Noetic安装，工作空间 `/home/rikibot/catkin_ws` 已构建。
- **环境**：SSH到机器人 (`rikibot@192.168.100.32`)，ROS_MASTER_URI 设置为 `http://192.168.100.32:11311`。
- **依赖**：确保 `teleop_twist_keyboard` 包安装（`sudo apt install ros-noetic-teleop-twist-keyboard`）。

## 完整流程步骤

### 步骤1: 检查环境和文件
- **目的**：确认工作空间和可执行文件存在。
- **命令**：
  ```
  cd /home/rikibot
  ls -la  # 查看根目录文件，如 catkin_ws/, start_*.sh 脚本
  cd catkin_ws
  ls src/  # 查看ROS包，如 rikibot_project/, lidar/, camera_umd/
  ls devel/lib/  # 查看可执行文件，如 rikibot_driver/, rplidar_ros/
  ```
- **逻辑**：确保catkin工作空间完整，包已编译。检查脚本用于快速启动。

### 步骤2: 验证ROS状态
- **目的**：确认ROS核心运行，检查话题。
- **命令**：
  ```
  rostopic list  # 列出活跃话题，如 /cmd_vel, /odom, /scan
  ```
- **逻辑**：如果无输出，需启动roscore。话题确认传感器数据流。

### 步骤3: 清理和设置环境（避免冲突）
- **目的**：杀掉旧进程，设置命名空间隔离。
- **命令**：
  ```
  killall -9 roscore roslaunch rosmaster rosout  # 杀掉所有ROS进程
  export ROS_NAMESPACE=rikibot_control  # 设置命名空间，避免节点冲突
  ```
- **逻辑**：清理防止重启冲突。命名空间使话题如 `/rikibot_control/cmd_vel`，隔离多机器人场景。

### 步骤4: 启动机器人基础系统
- **目的**：启动驱动、IMU、定位、TF变换。
- **命令**：
  ```
  roslaunch rikibot bringup.launch
  ```
- **逻辑**：启动核心节点（rikibot_driver订阅/cmd_vel，发布odom；IMU滤波；EKF定位）。在namespace下运行。输出显示节点启动，串口连接成功。

### 步骤5: 启动键盘控制
- **目的**：启用键盘输入发布运动命令。
- **命令**：
  ```
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  ```
- **逻辑**：节点发布Twist消息到 `/rikibot_control/cmd_vel`。在终端等待键盘输入。

### 步骤6: 使用键盘控制小车
- **目的**：实时控制运动。
- **操作**：
  - 在键盘控制终端按键：
    - **i**：前进
    - **k**：停止
    - **j**：左转
    - **l**：右转
    - **u/o**：斜向移动（全向模式按Shift）
    - **q/z**：增加/减少最大速度
    - **w/x**：调整线性速度
    - **e/c**：调整角速度
    - **Ctrl+C**：退出
- **逻辑**：键映射到Twist消息。小车响应/cmd_vel，执行运动。速度实时显示。

### 步骤7: 停止和清理
- **目的**：安全停止系统。
- **命令**：
  ```
  # 在键盘终端按 Ctrl+C 退出
  killall -9 roslaunch  # 杀掉launch进程
  ```
- **逻辑**：确保无残留进程，避免资源占用。

## 故障排除
- **串口错误**：`SerialException device disconnected` → 检查USB连接，运行 `ls /dev/rikibase` 确认设备，`sudo chmod 666 /dev/rikibase` 设置权限。
- **节点冲突**：`new node registered with same name` → 杀掉进程，设置namespace。
- **无运动响应**：检查 `rostopic echo /rikibot_control/cmd_vel` 是否接收消息；硬件电源/连接。
- **键盘无响应**：确保终端焦点在键盘控制节点上。
- **TF警告**：重复时间戳 → 通常无害，忽略或同步时钟。
- **包缺失**：`sudo apt install ros-noetic-teleop-twist-keyboard`。

## 注意事项
- **安全**：控制前确保周围无障碍，速度<0.5 m/s。
- **测试**：先在无硬件模式测试（`rostopic pub /cmd_vel ...`）。
- **日志**：查看 `~/.ros/log/` 调试错误。
- **版本**：适用于ROS Noetic，机器人rikibot-fx4。
- **快速启动**：使用脚本如 `start_best_3d_mapping.sh` 替代手动launch（但需修改为控制模式）。

## 示例完整命令序列
```
cd /home/rikibot
killall -9 roscore roslaunch rosmaster rosout
export ROS_NAMESPACE=rikibot_control

# 终端 1
roscore

# 终端 2
roslaunch rikibot bringup.launch 

# 终端 3
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# 按键控制，Ctrl+C退出
```

# RIKIBOT-FX4 小车雷达建图操作指南

## 概述
该部分描述如何通过ROS (Robot Operating System) 调用Rikibot-FX4小车现有搭载的思岚 rplidar c1 激光雷达以实现建图功能。适用于rikibot@robot环境。所有操作在终端中执行。

## 前提条件
- **硬件**：小车电源开启，USB连接正常（串口 `/dev/rikibase` 可用）；激光雷达UART接口已连接至主控。
- **软件**：ROS Noetic安装，工作空间 `/home/rikibot/catkin_ws` 已构建。
- **环境**：SSH到机器人 (`rikibot@192.168.100.32`)，ROS_MASTER_URI 设置为 `http://192.168.100.32:11311`。

## 完整流程步骤

### 步骤1-4: 参考前面的键盘控制指南的步骤1-4

### 步骤5: 启动雷达SLAM建图
- **目的**：启动雷达并使用slam_gmapping建图。
- **命令**：
  ```
  roslaunch rikibot lidar_slam.launch
  ```
- **逻辑**：节点会启动 rplidar_c1.launch 与 slam_gmapping.xml 并读取其中的参数，并开始建图。

### 步骤6: 启动Rviz
- **目的**：启动Rviz可视化工具窗口，可查看建图情况和更改配置。
- **命令**：
  ```
  export DISPLAY=本机IP:0.0  # 本机IP需自行查找，如192.168.100.91
  rviz # 打开rviz
  重点：在 File → Recent Configs 中选择 ~/catkin_ws/src/rikibot_project/rikibot/rviz/slam.rviz，否则无法进行可视化。
  ```
- **逻辑**：export 操作会启动SSH X11转发，将linux端（工控机端）的图形程序转发至Windows端（主控机端），rviz操作将打开rviz界面并在主控机端显示。 # 部署SSH X11转发的完整指南请参阅 " https://zhuanlan.zhihu.com/p/27155499043 "

### 步骤7-8: 参考上面的键盘控制指南部分的步骤5-6

### 步骤9: 保存所建的2D栅格地图
- **目的**：保存通过激光雷达扫描已经建立的2D栅格地图，后续可用于导航等功能。
- **命令**：
  ```
  rosrun map_server map_saver -f my_map # 保存地图的名称为my_map，同时保存其 
  .yaml 与 .pgm 文件。
  ```

## 示例完整命令序列
```
cd /home/rikibot
killall -9 roscore roslaunch rosmaster rosout
export ROS_NAMESPACE=rikibot_control

# 终端 1
roscore

# 终端 2
roslaunch rikibot bringup.launch 

# 终端 3
roslaunch rikibot lidar_slam.launch

# 终端 4
export DISPLAY=本机IP:0.0
rviz

# 终端 5
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# 终端 6
rosrun map_server map_saver -f my_map

# 按键控制，Ctrl+C退出
```

# RIKIBOT-FX4 小车导航操作指南

## 概述
该部分描述如何通过ROS (Robot Operating System) 调用Rikibot-FX4小车现有搭载的思岚 rplidar c1 激光雷达以实现导航功能。适用于rikibot@robot环境。所有操作在终端中执行。

## 前提条件
- **硬件**：小车电源开启，USB连接正常（串口 `/dev/rikibase` 可用）；激光雷达UART接口已连接至主控。
- **软件**：ROS Noetic安装，工作空间 `/home/rikibot/catkin_ws` 已构建。
- **环境**：SSH到机器人 (`rikibot@192.168.100.32`)，ROS_MASTER_URI 设置为 `http://192.168.100.32:11311`。

## 完整流程步骤

### 步骤1-4: 参考前面的键盘控制指南的步骤1-4

### 步骤5: 启动导航与急停服务
- **目的**：启动导航并同时启动Emergency Stop Service 急停服务。
- **命令**：
  ```
  roslaunch rikibot navigate.launch
  ```
- **逻辑**：节点会启动navigate.launch主程序，调用amcl.launch 进行重定位，调用 rplidar_c1.launch 与 slam_gmapping.xml 并读取其中的参数，并开始导航建图。

### 步骤6: 启动Rviz，参考前面雷达建图部分的步骤6

### 步骤7: 启动ROS服务调用急停/键盘控制急停
- **目的**：启动ROS服务调用急停或者键盘控制急停，可主动控制小车急停和重置急停状态，保证导航时小车的安全。
- **命令**：
  ```
  ROS服务调用：rosservice call /emergency_stop/trigger

  键盘控制：rosrun rikibot keyboard_emergency_stop.py
  ```
  注：详细使用说明和注意事项请参阅README文档，文档路径：/home/rikibot/catkin_ws/src/rikibot_project/rikibot/script/EMERGENCY_STOP_README.md
  ```

# RIKIBOT-FX4 小车自主探索建图导航操作指南

## 概述
该部分描述如何通过ROS (Robot Operating System) 调用Rikibot-FX4小车现有搭载的思岚 rplidar c1 激光雷达以实现导航功能。适用于rikibot@robot环境。所有操作在终端中执行。

## 前提条件
- **硬件**：小车电源开启，USB连接正常（串口 `/dev/rikibase` 可用）；激光雷达UART接口已连接至主控。
- **软件**：ROS Noetic安装，工作空间 `/home/rikibot/catkin_ws` 已构建。
- **环境**：SSH到机器人 (`rikibot@192.168.100.32`)，ROS_MASTER_URI 设置为 `http://192.168.100.32:11311`。

## 完整流程步骤

### 步骤1-4: 参考前面的键盘控制指南的步骤1-4

### 步骤5: 启动ROS服务调用急停/键盘控制急停
- **目的**：启动ROS服务调用急停或者键盘控制急停，可主动控制小车急停和重置急停状态，保证自主探索建图导航时小车的安全。
- **命令**：
  ```
  ROS服务调用：rosservice call /emergency_stop/trigger

  键盘控制：rosrun rikibot keyboard_emergency_stop.py
  ```
  注：详细使用说明和注意事项请参阅README文档，文档路径：/home/rikibot/catkin_ws/src/rikibot_project/rikibot/script/EMERGENCY_STOP_README.md
  ```
### 步骤6:参考上面的键盘控制指南部分的步骤5-6

### 步骤7: 启动自主探索建图导航
- **目的**：启动自主探索建图导航，机器人将自动进行环境探索、建图和导航。
- **命令**：
  ```
  roslaunch rikibot auto_slam.launch
  ```
- **逻辑**：节点会启动 auto_slam.launch 主程序，调用 rplidar_c1.launch 与 slam_gmapping.xml 并读取其中的参数，调用move_base.launch、explore_costmap.launch 以进行自主移动探索和避障，自启动slam.rviz可视化并开始自主探索建图导航。

## 示例完整命令序列
```
cd /home/rikibot
killall -9 roscore roslaunch rosmaster rosout
export ROS_NAMESPACE=rikibot_control

# 终端 1
roscore

# 终端 2
roslaunch rikibot bringup.launch 

# 终端 3
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# 终端 4
roslaunch rikibot auto_slam.launch

# 按键控制，Ctrl+C退出
```

此指南基于实际测试，确保AI能快速重现流程。
