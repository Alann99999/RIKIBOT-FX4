#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import subprocess
from collections import deque

import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


class MapCompletionWatcher(object):
    def __init__(self):
        self.state_pub = rospy.Publisher('~state', String, queue_size=10)

        self.status_publish_hz = rospy.get_param('~status_publish_hz', 1.0)
        self.command_linear_threshold = rospy.get_param('~command_linear_threshold', 0.05)
        self.command_angular_threshold = rospy.get_param('~command_angular_threshold', 0.20)
        self.completion_window = rospy.get_param('~completion_window', 180.0)
        self.completion_max_known_delta = rospy.get_param('~completion_max_known_delta', 20)
        self.completion_required_cycles = rospy.get_param('~completion_required_cycles', 8)
        self.idle_completion_timeout = rospy.get_param('~idle_completion_timeout', 60.0)
        self.min_completion_known_ratio = rospy.get_param('~min_completion_known_ratio', 15.0)
        self.active_goal_timeout = rospy.get_param('~active_goal_timeout', 45.0)
        self.map_save_path = rospy.get_param('~map_save_path', '/home/rikibot/catkin_ws/maps/whole_house')

        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.latest_map_time = None
        self.latest_map_known = 0
        self.latest_map_total = 0
        self.last_move_base_active_time = None
        self.last_move_base_state = 'unknown'
        self.last_motion_time = None
        self.map_history = deque()
        self.completion_candidate_count = 0
        self.mapping_completed = False
        self.saved_map_path = None

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=10)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=2)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback, queue_size=10)

        self.status_timer = rospy.Timer(rospy.Duration(1.0 / self.status_publish_hz), self.status_timer_callback)

    def cmd_vel_callback(self, msg):
        now = rospy.Time.now()
        self.latest_cmd = msg
        self.latest_cmd_time = now
        commanded_linear = abs(msg.linear.x) + abs(msg.linear.y)
        commanded_angular = abs(msg.angular.z)
        if commanded_linear >= self.command_linear_threshold or commanded_angular >= self.command_angular_threshold:
            self.last_motion_time = now

    def map_callback(self, msg):
        now = rospy.Time.now()
        known = 0
        for cell in msg.data:
            if cell >= 0:
                known += 1

        self.latest_map_time = now
        self.latest_map_known = known
        self.latest_map_total = len(msg.data)
        self.map_history.append((now, known))
        self.prune_history(now)

    def move_base_status_callback(self, msg):
        now = rospy.Time.now()
        codes = [status.status for status in msg.status_list]
        if any(code in (0, 1) for code in codes):
            self.last_move_base_state = 'active'
            self.last_move_base_active_time = now
        elif any(code == 3 for code in codes):
            self.last_move_base_state = 'succeeded'
        elif any(code in (4, 5, 9) for code in codes):
            self.last_move_base_state = 'blocked'
        elif codes:
            self.last_move_base_state = str(codes[-1])
        else:
            self.last_move_base_state = 'idle'

    def prune_history(self, now):
        max_window = self.completion_window + 30.0
        while self.map_history and (now - self.map_history[0][0]).to_sec() > max_window:
            self.map_history.popleft()

    def status_timer_callback(self, _event):
        now = rospy.Time.now()
        self.prune_history(now)
        self.evaluate(now)
        status = String()
        status.data = self.build_status(now)
        self.state_pub.publish(status)

    def evaluate(self, now):
        if self.mapping_completed:
            return
        if self.latest_map_time is None or self.latest_cmd_time is None:
            return
        if self.is_completion_candidate(now):
            self.completion_candidate_count += 1
            if self.completion_candidate_count >= self.completion_required_cycles:
                self.mapping_completed = True
                self.saved_map_path = self.save_map()
                rospy.loginfo('map_completion_watcher marked mapping complete')
        else:
            self.completion_candidate_count = 0

    def is_completion_candidate(self, now):
        if self.latest_map_total <= 0 or self.last_motion_time is None:
            return False
        known_ratio = 100.0 * float(self.latest_map_known) / float(self.latest_map_total)
        if known_ratio < self.min_completion_known_ratio:
            return False
        if (now - self.last_motion_time).to_sec() < self.idle_completion_timeout:
            return False
        if self.last_move_base_active_time is not None and (now - self.last_move_base_active_time).to_sec() < self.active_goal_timeout:
            return False
        return self.map_known_delta(now, self.completion_window) <= self.completion_max_known_delta

    def map_known_delta(self, now, window_seconds):
        history = list(self.map_history)
        if not history:
            return 0
        target_time = now - rospy.Duration(window_seconds)
        baseline = history[0][1]
        for stamp, known in history:
            if stamp <= target_time:
                baseline = known
            else:
                break
        return self.latest_map_known - baseline

    def build_status(self, now):
        if self.latest_map_time is None:
            return 'waiting_for_map'
        known_ratio = 0.0
        if self.latest_map_total > 0:
            known_ratio = 100.0 * float(self.latest_map_known) / float(self.latest_map_total)
        status = 'watching'
        if self.mapping_completed:
            status = 'mapping_completed'
        return '%s move_base=%s known=%d/%d(%.1f%%) map_delta=%d completion_checks=%d saved=%s' % (
            status,
            self.last_move_base_state,
            self.latest_map_known,
            self.latest_map_total,
            known_ratio,
            self.map_known_delta(now, self.completion_window),
            self.completion_candidate_count,
            self.saved_map_path if self.saved_map_path else 'none',
        )

    def save_map(self):
        directory = os.path.dirname(self.map_save_path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        target = self.map_save_path
        if os.path.exists(target + '.yaml') or os.path.exists(target + '.pgm'):
            target = '%s_%d' % (self.map_save_path, int(rospy.Time.now().to_sec()))
        try:
            subprocess.check_call(['rosrun', 'map_server', 'map_saver', '-f', target])
            rospy.loginfo('map_completion_watcher saved map to %s', target)
            return target
        except Exception as exc:
            rospy.logwarn('map_completion_watcher failed to save map: %s', exc)
            return None


if __name__ == '__main__':
    rospy.init_node('map_completion_watcher')
    MapCompletionWatcher()
    rospy.spin()