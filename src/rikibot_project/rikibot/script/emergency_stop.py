#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
RIKIBOT Emergency Stop Script
一键急停功能 - 立即停止机器人运动并取消导航目标
"""

import rospy
import sys
import signal
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String

class EmergencyStop:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('emergency_stop', anonymous=True)

        # 创建发布器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.status_pub = rospy.Publisher('/emergency_stop/status', String, queue_size=10)

        # 停止消息
        self.stop_cmd = Twist()
        self.cancel_msg = GoalID()

        # 状态标志
        self.is_stopped = False

        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        print("=== RIKIBOT 急停系统已启动 ===")
        print("按 Ctrl+C 激活急停")
        print("机器人将立即停止并取消当前导航目标")

    def emergency_stop(self):
        """执行急停操作"""
        if self.is_stopped:
            print("急停已激活")
            return

        print("\n!!! 急停激活 !!!")

        # 立即发布停止命令
        self.cmd_vel_pub.publish(self.stop_cmd)

        # 取消当前导航目标
        self.cancel_pub.publish(self.cancel_msg)

        # 持续发布停止命令一段时间，确保机器人完全停止
        stop_duration = 2.0  # 2秒
        stop_rate = 20  # 20Hz
        rate = rospy.Rate(stop_rate)

        for i in range(int(stop_duration * stop_rate)):
            self.cmd_vel_pub.publish(self.stop_cmd)
            rate.sleep()

        # 发布状态
        self.status_pub.publish("EMERGENCY_STOP_ACTIVATED")

        self.is_stopped = True
        print("机器人已停止，导航目标已取消")

    def signal_handler(self, signum, frame):
        """信号处理函数"""
        print("\n接收到停止信号，正在执行急停...")
        self.emergency_stop()
        sys.exit(0)

    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            # 发布状态
            if self.is_stopped:
                self.status_pub.publish("STOPPED")
            else:
                self.status_pub.publish("ACTIVE")

            rate.sleep()

if __name__ == '__main__':
    try:
        emergency_stop = EmergencyStop()
        emergency_stop.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n急停脚本已终止")