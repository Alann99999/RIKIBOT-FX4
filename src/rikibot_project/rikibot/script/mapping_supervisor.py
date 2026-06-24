#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import os
import subprocess
from collections import deque

import rospy
from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion


class MappingSupervisor(object):
    STATUS_LABELS = {
        0: 'pending',
        1: 'active',
        2: 'preempted',
        3: 'succeeded',
        4: 'aborted',
        5: 'rejected',
        6: 'preempting',
        7: 'recalling',
        8: 'recalled',
        9: 'lost',
    }

    def __init__(self):
        self.state_pub = rospy.Publisher('~state', String, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

        self.clear_costmaps_srv = None
        self.clear_costmaps_enabled = rospy.get_param('~clear_costmaps_enabled', True)
        self.clear_costmaps_timeout = rospy.get_param('~clear_costmaps_timeout', 1.0)
        self.command_linear_threshold = rospy.get_param('~command_linear_threshold', 0.05)
        self.command_angular_threshold = rospy.get_param('~command_angular_threshold', 0.25)
        self.progress_window = rospy.get_param('~progress_window', 20.0)
        self.no_progress_min_distance = rospy.get_param('~no_progress_min_distance', 0.08)
        self.min_known_cells_delta = rospy.get_param('~min_known_cells_delta', 25)
        self.map_stale_timeout = rospy.get_param('~map_stale_timeout', 30.0)
        self.active_goal_timeout = rospy.get_param('~active_goal_timeout', 20.0)
        self.recovery_cooldown = rospy.get_param('~recovery_cooldown', 12.0)
        self.stop_publish_hz = rospy.get_param('~stop_publish_hz', 15.0)
        self.status_publish_hz = rospy.get_param('~status_publish_hz', 1.0)
        self.recovery_window = rospy.get_param('~recovery_window', 180.0)
        self.recovery_confirmation_count = rospy.get_param('~recovery_confirmation_count', 3)
        self.repeated_recovery_limit = rospy.get_param('~repeated_recovery_limit', 8)
        self.kill_explore_on_repeated_recoveries = rospy.get_param('~kill_explore_on_repeated_recoveries', False)
        self.completion_window = rospy.get_param('~completion_window', 120.0)
        self.completion_max_known_delta = rospy.get_param('~completion_max_known_delta', 20)
        self.completion_required_cycles = rospy.get_param('~completion_required_cycles', 5)
        self.idle_completion_timeout = rospy.get_param('~idle_completion_timeout', 45.0)
        self.min_completion_known_ratio = rospy.get_param('~min_completion_known_ratio', 15.0)
        self.save_map_on_completion = rospy.get_param('~save_map_on_completion', True)
        self.stop_on_completion = rospy.get_param('~stop_on_completion', True)
        self.map_save_path = rospy.get_param('~map_save_path', '/home/rikibot/catkin_ws/maps/whole_house')

        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.latest_pose = None
        self.latest_odom_time = None
        self.latest_map_time = None
        self.latest_map_known = 0
        self.latest_map_occupied = 0
        self.latest_map_total = 0
        self.last_guard_state = 'unknown'
        self.last_move_base_state = 'unknown'
        self.last_move_base_active_time = None
        self.last_motion_time = None
        self.last_status_message = 'waiting_for_data'
        self.recovery_cooldown_until = rospy.Time(0)
        self.pose_history = deque()
        self.map_history = deque()
        self.recovery_times = deque()
        self.pending_recovery_reason = None
        self.pending_recovery_count = 0
        self.completion_candidate_count = 0
        self.mapping_completed = False
        self.saved_map_path = None

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=50)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=2)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback, queue_size=10)
        rospy.Subscriber('/navigation_guard/state', String, self.guard_state_callback, queue_size=10)

        self.stop_timer = rospy.Timer(rospy.Duration(1.0 / self.stop_publish_hz), self.stop_timer_callback)
        self.status_timer = rospy.Timer(rospy.Duration(1.0 / self.status_publish_hz), self.status_timer_callback)

    def cmd_vel_callback(self, msg):
        self.latest_cmd = msg
        now = rospy.Time.now()
        self.latest_cmd_time = now
        commanded_linear = abs(msg.linear.x) + abs(msg.linear.y)
        commanded_angular = abs(msg.angular.z)
        if commanded_linear >= self.command_linear_threshold or commanded_angular >= self.command_angular_threshold:
            self.last_motion_time = now

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
        self.prune_history(now)

    def map_callback(self, msg):
        now = rospy.Time.now()
        known = 0
        occupied = 0
        for cell in msg.data:
            if cell >= 0:
                known += 1
            if cell > 50:
                occupied += 1

        self.latest_map_time = now
        self.latest_map_known = known
        self.latest_map_occupied = occupied
        self.latest_map_total = len(msg.data)
        self.map_history.append((now, known, occupied))
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
            labels = [self.STATUS_LABELS.get(code, str(code)) for code in codes if code in (4, 5, 9)]
            self.last_move_base_state = '+'.join(labels[:2]) if labels else 'blocked'
        elif codes:
            self.last_move_base_state = self.STATUS_LABELS.get(codes[-1], str(codes[-1]))
        else:
            self.last_move_base_state = 'idle'

    def guard_state_callback(self, msg):
        self.last_guard_state = msg.data

    def prune_history(self, now):
        max_window = max(self.progress_window, self.recovery_window, self.map_stale_timeout) + 5.0
        while self.pose_history and (now - self.pose_history[0][3]).to_sec() > max_window:
            self.pose_history.popleft()
        while self.map_history and (now - self.map_history[0][0]).to_sec() > max_window:
            self.map_history.popleft()
        while self.recovery_times and (now - self.recovery_times[0]).to_sec() > self.recovery_window:
            self.recovery_times.popleft()

    def stop_timer_callback(self, _event):
        if rospy.Time.now() < self.recovery_cooldown_until:
            self.publish_stop()

    def status_timer_callback(self, _event):
        now = rospy.Time.now()
        self.prune_history(now)
        self.evaluate(now)
        status = String()
        status.data = self.build_status_message(now)
        self.last_status_message = status.data
        self.state_pub.publish(status)

    def evaluate(self, now):
        if self.mapping_completed:
            return
        if self.latest_cmd_time is None or self.latest_odom_time is None or self.latest_map_time is None:
            return
        if now < self.recovery_cooldown_until:
            return
        if self.guard_is_busy():
            self.completion_candidate_count = 0
            self.clear_pending_recovery()
            return

        if self.is_completion_candidate(now):
            self.completion_candidate_count += 1
            if self.completion_candidate_count >= self.completion_required_cycles:
                self.handle_completion()
            return

        self.completion_candidate_count = 0
        if not self.has_active_navigation(now):
            self.clear_pending_recovery()
            return

        map_age = (now - self.latest_map_time).to_sec()
        progress_distance = self.progress_distance(now)
        known_delta = self.map_known_delta(now)

        if map_age > self.map_stale_timeout:
            self.register_recovery_candidate('map_stale')
            return

        if progress_distance < self.no_progress_min_distance and known_delta < self.min_known_cells_delta:
            self.register_recovery_candidate('low_progress')
            return

        self.clear_pending_recovery()

    def register_recovery_candidate(self, reason):
        if self.pending_recovery_reason == reason:
            self.pending_recovery_count += 1
        else:
            self.pending_recovery_reason = reason
            self.pending_recovery_count = 1

        if self.pending_recovery_count >= self.recovery_confirmation_count:
            self.handle_recovery(reason)
            self.clear_pending_recovery()

    def clear_pending_recovery(self):
        self.pending_recovery_reason = None
        self.pending_recovery_count = 0

    def handle_recovery(self, reason):
        now = rospy.Time.now()
        self.recovery_times.append(now)
        self.recovery_cooldown_until = now + rospy.Duration(self.recovery_cooldown)
        self.last_status_message = 'recovery:%s' % reason
        rospy.logwarn('mapping_supervisor detected %s, stopping robot, canceling goal, and clearing costmaps', reason)
        self.publish_stop()
        self.cancel_pub.publish(GoalID())
        self.try_clear_costmaps()

        if len(self.recovery_times) >= self.repeated_recovery_limit and self.kill_explore_on_repeated_recoveries:
            self.kill_node('/explore')
            self.last_status_message = 'autonomy_paused'

    def handle_completion(self):
        self.mapping_completed = True
        self.last_status_message = 'mapping_completed'
        rospy.loginfo('mapping_supervisor detected stable map completion')
        self.publish_stop()
        self.cancel_pub.publish(GoalID())
        if self.save_map_on_completion:
            self.saved_map_path = self.save_map()
        if self.stop_on_completion:
            self.kill_node('/explore')

    def is_completion_candidate(self, now):
        if self.latest_map_total <= 0:
            return False
        known_ratio = 100.0 * float(self.latest_map_known) / float(self.latest_map_total)
        if known_ratio < self.min_completion_known_ratio:
            return False
        if self.last_motion_time is None:
            return False
        if (now - self.last_motion_time).to_sec() < self.idle_completion_timeout:
            return False
        if self.has_active_navigation(now):
            return False
        return self.map_known_delta(now, self.completion_window) <= self.completion_max_known_delta

    def build_status_message(self, now):
        if self.latest_map_time is None or self.latest_odom_time is None or self.latest_cmd_time is None:
            return 'waiting_for_data'

        map_age = (now - self.latest_map_time).to_sec()
        progress_distance = self.progress_distance(now)
        known_delta = self.map_known_delta(now)
        known_ratio = 0.0
        if self.latest_map_total > 0:
            known_ratio = 100.0 * float(self.latest_map_known) / float(self.latest_map_total)

        commanded_linear = abs(self.latest_cmd.linear.x) + abs(self.latest_cmd.linear.y)
        commanded_angular = abs(self.latest_cmd.angular.z)
        status_label = 'monitoring'
        if self.mapping_completed:
            status_label = 'mapping_completed'
        elif now < self.recovery_cooldown_until:
            status_label = self.last_status_message + ':cooldown'
        elif not self.has_active_navigation(now):
            status_label = 'waiting_goal'

        return (
            '%s move_base=%s guard=%s map_age=%.1fs known=%d/%d(%.1f%%) '
            'map_delta=%d progress=%.3fm cmd_linear=%.3f cmd_angular=%.3f recoveries=%d completion_checks=%d'
        ) % (
            status_label,
            self.last_move_base_state,
            self.last_guard_state,
            map_age,
            self.latest_map_known,
            self.latest_map_total,
            known_ratio,
            known_delta,
            progress_distance,
            commanded_linear,
            commanded_angular,
            len(self.recovery_times),
            self.completion_candidate_count,
        )

    def has_active_navigation(self, now):
        if self.last_move_base_active_time is None:
            return False
        if (now - self.last_move_base_active_time).to_sec() > self.active_goal_timeout:
            return False
        commanded_linear = abs(self.latest_cmd.linear.x) + abs(self.latest_cmd.linear.y)
        commanded_angular = abs(self.latest_cmd.angular.z)
        return (
            self.last_move_base_state == 'active'
            or commanded_linear >= self.command_linear_threshold
            or commanded_angular >= self.command_angular_threshold
        )

    def guard_is_busy(self):
        return self.last_guard_state.startswith('event:') or self.last_guard_state == 'autonomy_paused'

    def progress_distance(self, now):
        start = self.get_pose_at_age(now, self.progress_window)
        end = self.latest_pose
        if start is None or end is None:
            return 0.0
        return math.hypot(end[0] - start[0], end[1] - start[1])

    def map_known_delta(self, now, window_seconds=None):
        history = list(self.map_history)
        if not history:
            return 0
        target_window = self.progress_window if window_seconds is None else window_seconds
        target_time = now - rospy.Duration(target_window)
        baseline = history[0][1]
        for item in history:
            if item[0] <= target_time:
                baseline = item[1]
            else:
                break
        return self.latest_map_known - baseline

    def get_pose_at_age(self, now, window_seconds):
        history = list(self.pose_history)
        target_time = now - rospy.Duration(window_seconds)
        candidate = None
        for pose in history:
            if pose[3] <= target_time:
                candidate = pose
            else:
                break
        return candidate if candidate is not None else (history[0] if history else None)

    def save_map(self):
        directory = os.path.dirname(self.map_save_path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        target = self.map_save_path
        if os.path.exists(target + '.yaml') or os.path.exists(target + '.pgm'):
            stamp = rospy.Time.now().to_sec()
            target = '%s_%d' % (self.map_save_path, int(stamp))
        try:
            subprocess.check_call(['rosrun', 'map_server', 'map_saver', '-f', target])
            rospy.loginfo('mapping_supervisor saved map to %s', target)
            return target
        except Exception as exc:
            rospy.logwarn('mapping_supervisor failed to save map: %s', exc)
            return None

    def try_clear_costmaps(self):
        if not self.clear_costmaps_enabled:
            return
        try:
            if self.clear_costmaps_srv is None:
                rospy.wait_for_service('/move_base/clear_costmaps', timeout=self.clear_costmaps_timeout)
                self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            self.clear_costmaps_srv()
            rospy.loginfo('mapping_supervisor cleared move_base costmaps')
        except Exception as exc:
            rospy.logwarn('mapping_supervisor failed to clear costmaps: %s', exc)

    def kill_node(self, node_name):
        try:
            subprocess.call(['rosnode', 'kill', node_name])
            rospy.logwarn('mapping_supervisor killed %s after repeated recoveries', node_name)
        except Exception as exc:
            rospy.logwarn('mapping_supervisor failed to kill %s: %s', node_name, exc)

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
    rospy.init_node('mapping_supervisor')
    MappingSupervisor()
    rospy.spin()