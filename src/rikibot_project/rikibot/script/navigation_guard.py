#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import subprocess
from collections import deque

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion


class NavigationGuard(object):
    def __init__(self):
        self.state_pub = rospy.Publisher('~state', String, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

        self.clear_costmaps_srv = None
        self.clear_costmaps_enabled = rospy.get_param('~clear_costmaps_enabled', True)
        self.clear_costmaps_timeout = rospy.get_param('~clear_costmaps_timeout', 1.0)

        self.command_linear_threshold = rospy.get_param('~command_linear_threshold', 0.08)
        self.command_angular_threshold = rospy.get_param('~command_angular_threshold', 0.45)
        self.stuck_window = rospy.get_param('~stuck_window', 4.0)
        self.stuck_min_progress = rospy.get_param('~stuck_min_progress', 0.06)
        self.spin_window = rospy.get_param('~spin_window', 8.0)
        self.spin_min_yaw_change = rospy.get_param('~spin_min_yaw_change', 4.5)
        self.spin_max_distance = rospy.get_param('~spin_max_distance', 0.18)
        self.cooldown_after_event = rospy.get_param('~cooldown_after_event', 5.0)
        self.stop_publish_hz = rospy.get_param('~stop_publish_hz', 15.0)
        self.status_publish_hz = rospy.get_param('~status_publish_hz', 1.0)
        self.event_window = rospy.get_param('~event_window', 60.0)
        self.event_confirmation_count = rospy.get_param('~event_confirmation_count', 3)
        self.repeated_event_limit = rospy.get_param('~repeated_event_limit', 3)
        self.kill_explore_on_repeated_events = rospy.get_param('~kill_explore_on_repeated_events', True)
        self.kill_move_base_on_repeated_events = rospy.get_param('~kill_move_base_on_repeated_events', False)

        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.latest_odom_time = None
        self.latest_pose = None
        self.last_state_message = 'idle'
        self.cooldown_until = rospy.Time(0)
        self.event_times = deque()
        self.pose_history = deque()
        self.guard_disabled = False
        self.pending_event_name = None
        self.pending_event_count = 0

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=50)

        self.stop_timer = rospy.Timer(rospy.Duration(1.0 / self.stop_publish_hz), self.stop_timer_callback)
        self.status_timer = rospy.Timer(rospy.Duration(1.0 / self.status_publish_hz), self.status_timer_callback)

    def cmd_vel_callback(self, msg):
        self.latest_cmd = msg
        self.latest_cmd_time = rospy.Time.now()

    def odom_callback(self, msg):
        now = rospy.Time.now()
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        if self.latest_pose is None:
            unwrapped_yaw = yaw
        else:
            prev_yaw = self.latest_pose[2]
            delta = yaw - self.wrap_angle(prev_yaw)
            delta = math.atan2(math.sin(delta), math.cos(delta))
            unwrapped_yaw = prev_yaw + delta

        pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            unwrapped_yaw,
            now,
        )
        self.latest_pose = pose
        self.latest_odom_time = now
        self.pose_history.append(pose)
        self.prune_pose_history(now)
        self.evaluate_anomaly(now)

    def prune_pose_history(self, now):
        max_window = max(self.stuck_window, self.spin_window, self.event_window) + 2.0
        while self.pose_history and (now - self.pose_history[0][3]).to_sec() > max_window:
            self.pose_history.popleft()
        while self.event_times and (now - self.event_times[0]).to_sec() > self.event_window:
            self.event_times.popleft()

    def evaluate_anomaly(self, now):
        if self.guard_disabled or self.latest_cmd_time is None or self.latest_pose is None:
            return
        if (now - self.latest_cmd_time).to_sec() > 1.0:
            return
        if now < self.cooldown_until:
            return

        commanded_linear = abs(self.latest_cmd.linear.x) + abs(self.latest_cmd.linear.y)
        commanded_angular = abs(self.latest_cmd.angular.z)

        if commanded_linear >= self.command_linear_threshold and self.is_stuck(now):
            self.register_event_candidate('stuck')
            return

        if commanded_angular >= self.command_angular_threshold and self.is_spinning(now):
            self.register_event_candidate('spinning')
            return

        self.clear_pending_event()

    def register_event_candidate(self, event_name):
        if self.pending_event_name == event_name:
            self.pending_event_count += 1
        else:
            self.pending_event_name = event_name
            self.pending_event_count = 1

        if self.pending_event_count >= self.event_confirmation_count:
            self.handle_event(event_name)
            self.clear_pending_event()

    def clear_pending_event(self):
        self.pending_event_name = None
        self.pending_event_count = 0

    def is_stuck(self, now):
        start = self.get_pose_at_age(now, self.stuck_window)
        end = self.latest_pose
        if start is None or end is None:
            return False
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        return distance < self.stuck_min_progress

    def is_spinning(self, now):
        start = self.get_pose_at_age(now, self.spin_window)
        end = self.latest_pose
        if start is None or end is None:
            return False
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        yaw_change = abs(end[2] - start[2])
        return distance < self.spin_max_distance and yaw_change > self.spin_min_yaw_change

    def get_pose_at_age(self, now, window_seconds):
        target_time = now - rospy.Duration(window_seconds)
        candidate = None
        for pose in self.pose_history:
            if pose[3] <= target_time:
                candidate = pose
            else:
                break
        return candidate if candidate is not None else (self.pose_history[0] if self.pose_history else None)

    def handle_event(self, event_name):
        now = rospy.Time.now()
        self.event_times.append(now)
        self.cooldown_until = now + rospy.Duration(self.cooldown_after_event)
        self.publish_stop()
        self.cancel_pub.publish(GoalID())
        self.last_state_message = 'event:%s' % event_name
        rospy.logwarn('navigation_guard detected %s, stopping robot and canceling goal', event_name)
        self.try_clear_costmaps()

        if len(self.event_times) >= self.repeated_event_limit:
            rospy.logwarn('navigation_guard observed repeated anomalies (%d in %.1fs)', len(self.event_times), self.event_window)
            if self.kill_explore_on_repeated_events:
                self.kill_node('/explore')
            if self.kill_move_base_on_repeated_events:
                self.kill_node('/move_base')
            self.guard_disabled = self.kill_explore_on_repeated_events or self.kill_move_base_on_repeated_events
            if self.guard_disabled:
                self.last_state_message = 'autonomy_paused'

    def try_clear_costmaps(self):
        if not self.clear_costmaps_enabled:
            return
        try:
            if self.clear_costmaps_srv is None:
                rospy.wait_for_service('/move_base/clear_costmaps', timeout=self.clear_costmaps_timeout)
                self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            self.clear_costmaps_srv()
            rospy.loginfo('navigation_guard cleared move_base costmaps')
        except Exception as exc:
            rospy.logwarn('navigation_guard failed to clear costmaps: %s', exc)

    def kill_node(self, node_name):
        try:
            subprocess.call(['rosnode', 'kill', node_name])
            rospy.logwarn('navigation_guard killed %s for safety', node_name)
        except Exception as exc:
            rospy.logwarn('navigation_guard failed to kill %s: %s', node_name, exc)

    def stop_timer_callback(self, _event):
        if rospy.Time.now() < self.cooldown_until:
            self.publish_stop()

    def status_timer_callback(self, _event):
        status = String()
        status.data = self.build_status_message()
        self.state_pub.publish(status)

    def build_status_message(self):
        now = rospy.Time.now()
        if self.guard_disabled:
            return 'autonomy_paused'
        if now < self.cooldown_until:
            return self.last_state_message + ':cooldown'
        if self.latest_cmd_time is None or self.latest_odom_time is None:
            return 'waiting_for_data'
        commanded_linear = abs(self.latest_cmd.linear.x) + abs(self.latest_cmd.linear.y)
        commanded_angular = abs(self.latest_cmd.angular.z)
        return 'monitoring cmd_linear=%.3f cmd_angular=%.3f events=%d' % (
            commanded_linear,
            commanded_angular,
            len(self.event_times),
        )

    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    @staticmethod
    def quaternion_to_yaw(orientation):
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


if __name__ == '__main__':
    rospy.init_node('navigation_guard')
    NavigationGuard()
    rospy.spin()
