# RIKIBOT 急停系统使用说明

## 概述
RIKIBOT急停系统提供多种方式在机器人即将碰撞时立即停止其运动，保障安全。

## 功能组件

### 1. 急停服务 (emergency_stop_service.py)
- **启动方式**: 随导航launch文件自动启动
- **服务名称**: `/emergency_stop/trigger`
- **调用方法**:
  ```bash
  rosservice call /emergency_stop/trigger
  ```
- **状态话题**: `/emergency_stop/status`

### 2. 键盘急停 (keyboard_emergency_stop.py)
- **启动方法**:
  ```bash
  rosrun rikibot keyboard_emergency_stop.py
  ```
- **控制键**:
  - `空格键` 或 `s`: 激活急停
  - `q`: 退出程序
  - `r`: 重置急停状态
  - `Ctrl+C`: 紧急停止并退出

### 3. 独立急停脚本 (emergency_stop.py)
- **启动方法**:
  ```bash
  rosrun rikibot emergency_stop.py
  ```
- **功能**: 启动后按 `Ctrl+C` 激活急停

## 使用方法

### 基本使用
1. 启动导航系统:
   ```bash
   roslaunch rikibot navigate.launch
   ```

2. 激活急停（选择一种方式）:
   - **方法1**: 调用ROS服务
     ```bash
     rosservice call /emergency_stop/trigger
     ```
   - **方法2**: 启动键盘控制
     ```bash
     rosrun rikibot keyboard_emergency_stop.py
     ```
     然后按空格键

### 集成到其他系统
急停服务可以轻松集成到其他ROS节点或外部系统中：

```python
import rospy
from std_srvs.srv import Trigger

# 调用急停服务
rospy.wait_for_service('/emergency_stop/trigger')
emergency_stop = rospy.ServiceProxy('/emergency_stop/trigger', Trigger)
response = emergency_stop()
```

## 急停机制
激活急停时，系统会：
1. 立即向 `/cmd_vel` 话题发布零速度命令
2. 取消当前的move_base导航目标
3. 持续发布停止命令2秒，确保机器人完全停止
4. 发布急停状态信息

## 安全注意事项
- 急停功能会立即停止机器人，可能导致 abrupt 停止
- 建议在低速导航时使用急停功能
- 急停后需要手动重新设置导航目标
- 可以根据需要调整停止持续时间和频率

## 故障排除
- 如果急停服务不可用，脚本会自动降级为直接发布停止命令
- 检查ROS网络连接确保服务调用正常
- 查看 `/emergency_stop/status` 话题了解当前状态</content>
<parameter name="filePath">/home/rikibot/catkin_ws/src/rikibot_project/rikibot/script/EMERGENCY_STOP_README.md