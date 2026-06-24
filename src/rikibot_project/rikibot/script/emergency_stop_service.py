#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
RIKIBOT Emergency Stop Service
急停服务 - 通过ROS服务调用实现一键急停
"""

import rospy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

class EmergencyStopService:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('emergency_stop_service')

        # 创建发布器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.status_pub = rospy.Publisher('/emergency_stop/status', String, queue_size=10)

        # 创建服务
        self.stop_service = rospy.Service('/emergency_stop/trigger', Trigger, self.emergency_stop_callback)

        # 停止消息
        self.stop_cmd = Twist()
        self.cancel_msg = GoalID()

        # 状态标志
        self.is_stopped = False
        self.last_stop_time = rospy.Time(0)

        print("=== RIKIBOT 急停服务已启动 ===")
        print("调用服务: rosservice call /emergency_stop/trigger")
        print("或发布话题: rostopic pub /emergency_stop/trigger std_msgs/Empty")

    def emergency_stop_callback(self, request):
        """急停服务回调函数"""
        if self.is_stopped:
            return TriggerResponse(
                success=False,
                message="急停已激活，机器人已停止"
            )

        print("\n!!! 急停激活 !!!")

        # 记录停止时间
        self.last_stop_time = rospy.Time.now()

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

        return TriggerResponse(
            success=True,
            message="急停成功激活，机器人已停止"
        )

    def reset_stop(self):
        """重置停止状态（可选，用于允许机器人重新启动）"""
        if self.is_stopped and (rospy.Time.now() - self.last_stop_time).to_sec() > 5.0:
            self.is_stopped = False
            self.status_pub.publish("RESET")
            print("急停状态已重置")

    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            # 定期重置停止状态（如果需要）
            self.reset_stop()

            # 发布当前状态
            if self.is_stopped:
                self.status_pub.publish("STOPPED")
            else:
                self.status_pub.publish("ACTIVE")

            rate.sleep()

if __name__ == '__main__':
    try:
        emergency_stop_service = EmergencyStopService()
        emergency_stop_service.run()
    except rospy.ROSInterruptException:
        pass