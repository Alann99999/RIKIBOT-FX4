#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
RIKIBOT Keyboard Emergency Stop
键盘急停脚本 - 按空格键或特定键激活急停
"""

import rospy
import sys
import termios
import tty
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from std_srvs.srv import Trigger

class KeyboardEmergencyStop:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('keyboard_emergency_stop', anonymous=True)

        # 创建发布器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

        # 停止消息
        self.stop_cmd = Twist()
        self.cancel_msg = GoalID()

        # 等待急停服务
        try:
            rospy.wait_for_service('/emergency_stop/trigger', timeout=5.0)
            self.emergency_stop_srv = rospy.ServiceProxy('/emergency_stop/trigger', Trigger)
            print("急停服务连接成功")
        except:
            print("警告：急停服务不可用，将直接发布停止命令")
            self.emergency_stop_srv = None

        print("\n=== RIKIBOT 键盘急停控制 ===")
        print("按 '空格键' 或 's' 键激活急停")
        print("按 'q' 键退出")
        print("按 'r' 键重置急停状态")
        print("------------------------------")

    def get_key(self):
        """获取键盘输入"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def emergency_stop(self):
        """执行急停"""
        print("\n!!! 急停激活 !!!")

        # 尝试调用急停服务
        if self.emergency_stop_srv:
            try:
                response = self.emergency_stop_srv()
                if response.success:
                    print("急停服务响应:", response.message)
                else:
                    print("急停服务错误:", response.message)
            except:
                print("急停服务调用失败，直接发布停止命令")
                self.direct_stop()
        else:
            self.direct_stop()

    def direct_stop(self):
        """直接发布停止命令"""
        # 立即发布停止命令
        self.cmd_vel_pub.publish(self.stop_cmd)

        # 取消当前导航目标
        self.cancel_pub.publish(self.cancel_msg)

        # 持续发布停止命令一段时间
        stop_duration = 2.0
        stop_rate = 20
        rate = rospy.Rate(stop_rate)

        for i in range(int(stop_duration * stop_rate)):
            self.cmd_vel_pub.publish(self.stop_cmd)
            rate.sleep()

        print("机器人已停止，导航目标已取消")

    def reset_stop(self):
        """重置急停状态"""
        print("急停状态已重置")

    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            try:
                key = self.get_key()

                if key == ' ':  # 空格键
                    self.emergency_stop()
                elif key.lower() == 's':  # s键
                    self.emergency_stop()
                elif key.lower() == 'q':  # q键退出
                    print("\n退出键盘急停控制")
                    break
                elif key.lower() == 'r':  # r键重置
                    self.reset_stop()
                elif key == '\x03':  # Ctrl+C
                    print("\n接收到Ctrl+C，正在执行急停...")
                    self.emergency_stop()
                    break

            except KeyboardInterrupt:
                print("\n接收到中断信号，正在执行急停...")
                self.emergency_stop()
                break

if __name__ == '__main__':
    try:
        keyboard_stop = KeyboardEmergencyStop()
        keyboard_stop.run()
    except rospy.ROSInterruptException:
        pass