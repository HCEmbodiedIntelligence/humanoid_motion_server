"""Terminal keyboard control of a configured ServoP channel."""

import argparse
import copy
import curses
import json
import math
import select
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .cartesian_debug import fresh, pose_error, quaternion, rpy_degrees, step_pose, transform
from .simulation.configuration import load_configuration


QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
# axis, sign, angular: ROS reference X forward, Y left, Z up.
KEYS = {key: (axis, sign, angular)
        for angular, pairs in ((False, ('ws', 'ad', 'rf')), (True, ('uj', 'ik', 'ol')))
        for axis, pair in enumerate(pairs) for key, sign in zip(pair, (1, -1))}


def select_channel(channels, *, group=None, name=None):
    candidates = [c for c in channels if c['kind'] == 'servo_p' and c.get('fk_pose_topic')
                  and (group is None or c['group'] == group) and (name is None or c['name'] == name)]
    if not candidates:
        raise ValueError('没有匹配且配置了 fk_pose_topic 的 ServoP 通道')
    if len(candidates) > 1:
        names = ', '.join(c['name'] for c in candidates)
        raise ValueError(f'存在多个 ServoP 通道，请用 --group 或 --channel 选择: {names}')
    return candidates[0]


class KeyboardServoNode(Node):
    def __init__(self, configuration, channel, args):
        super().__init__('keyboard_servo_p')
        self.configuration, self.channel, self.args = configuration, channel, args
        self.names = set(configuration['motion'][f"groups.{channel['group']}"])
        self.pose = self.state = self.observed_target = self.target = None
        self.simulation = {}
        self.simulation_seen = float('-inf')
        self.armed, self.local, self.done = False, False, False
        self.last_publish = None
        self.scale = 1.0
        self.notice = 'Enter 从当前 FK 开始；Space 暂停；Q 退出'
        self.lease = configuration['motion'].get('servo_lease_ms', 100) / 1000.0
        self.feedback_age = min(configuration['motion'].get('feedback_max_age_ms', 100)/1000.0, 0.25)
        self.publisher = self.create_publisher(PoseStamped, channel['endpoint'], QOS)
        self.create_subscription(PoseStamped, channel['fk_pose_topic'], self.receive_pose, QOS)
        self.create_subscription(PoseStamped, channel['endpoint'],
                                 lambda m: setattr(self, 'observed_target', m), QOS)
        self.create_subscription(JointState, configuration['motion']['joint_state_endpoint'],
                                 lambda m: setattr(self, 'state', m), QOS)
        self.create_subscription(String, '/simulation/status', self.receive_simulation, QOS)
        self.create_timer(1.0 / args.rate, self.publish_target)

    def receive_pose(self, message):
        self.pose = message

    def receive_simulation(self, message):
        try:
            status = json.loads(message.data)
            if isinstance(status, dict):
                self.simulation, self.simulation_seen = status, time.monotonic()
        except (ValueError, TypeError):
            pass

    def feedback_reason(self):
        now = self.get_clock().now().nanoseconds
        if not self.args.allow_hardware and (
                time.monotonic() - self.simulation_seen > 2.5 or
                self.simulation.get('mode') != 'kinematic_simulation' or
                self.simulation.get('robot_id') != self.configuration['robot_id']):
            return '等待当前机器人 /simulation/status；默认仅允许仿真'
        if not fresh(self.pose, now, self.feedback_age):
            return 'FK 反馈过期或尚未收到'
        try:
            transform(self.pose)
        except ValueError as error:
            return str(error)
        if self.pose.header.frame_id != self.channel['base_frame']:
            return f"FK 坐标系不匹配: {self.pose.header.frame_id} != {self.channel['base_frame']}"
        state = self.state
        if not fresh(state, now, self.feedback_age):
            return '关节反馈过期或尚未收到'
        if (len(state.name) != len(state.position) or len(state.name) != len(state.velocity)
                or len(set(state.name)) != len(state.name) or not all(state.name)
                or not self.names.issubset(state.name)
                or not all(math.isfinite(v) for values in (state.position, state.velocity) for v in values)):
            return '关节反馈字段或数值无效'
        if self.publisher.get_subscription_count() == 0:
            return 'ServoP 尚无订阅者'
        return ''

    def pause(self, reason='已暂停发布；按 Enter 从当前 FK 重新开始'):
        self.armed = False
        self.last_publish = None
        self.notice = reason

    def rebase(self):
        reason = self.feedback_reason()
        if reason:
            self.pause(reason)
            return False
        self.target = copy.deepcopy(self.pose)
        q = self.target.pose.orientation
        q.x, q.y, q.z, q.w = map(float, quaternion(self.pose.pose))
        return True

    def handle_key(self, key):
        if key in ('q', 'Q', '\x03'):
            self.pause('已停止发布')
            self.done = True
        elif key == ' ':
            self.pause()
        elif key in ('\n', '\r'):
            if self.rebase():
                self.armed = True
                self.last_publish = time.monotonic()
                self.notice = '发布中；每次按键改变一步，长按使用系统连发'
        elif key.lower() == 't':
            self.local = not self.local
            self.notice = '使用目标工具自身轴' if self.local else '使用固定参考坐标系轴'
        elif key in ('+', '='):
            self.scale = min(4.0, self.scale * 2)
        elif key == '-':
            self.scale = max(0.125, self.scale / 2)
        elif key.lower() == 'c':
            if self.rebase():
                self.notice = '目标已对齐当前实测 FK'
        elif key.lower() in KEYS:
            if not self.armed:
                self.notice = '已暂停；按 Enter 从当前 FK 开始后再移动'
                return
            reason = self.feedback_reason()
            if reason:
                self.pause(reason)
                return
            axis, sign, angular = KEYS[key.lower()]
            step = math.radians(self.args.angle_step) if angular else self.args.step
            candidate = step_pose(self.target, axis, sign*step*self.scale, angular=angular, local=self.local)
            distance, angle = pose_error(transform(self.pose), transform(candidate))
            if distance > self.args.max_offset or angle > math.radians(self.args.max_angle):
                self.notice = '这一步超出目标相对实测的偏差上限；等待跟随或按 C 对齐'
                return
            self.target = candidate
            self.notice = f"{'旋转' if angular else '平移'} {'XYZ'[axis]} {'+' if sign > 0 else '-'}"

    def publish_target(self):
        if not self.armed:
            return
        now = time.monotonic()
        if self.last_publish is not None and now-self.last_publish >= self.lease:
            self.pause('发布中断超过 Servo lease；按 Enter 重新开始')
            return
        reason = self.feedback_reason()
        if reason:
            self.pause(reason)
            return
        # Frame changes or invalid target edits must never silently change a command's meaning.
        try:
            transform(self.target)
            if self.target.header.frame_id != self.pose.header.frame_id:
                raise ValueError('目标与 FK 坐标系不一致')
        except ValueError as error:
            self.pause(str(error))
            return
        self.target.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.target)
        self.last_publish = now


def pose_lines(label, message):
    if message is None:
        return [f'{label}: waiting for pose']
    try:
        xyz = transform(message)[:3, 3]
        q, rpy = quaternion(message.pose), rpy_degrees(message)
        return [f'{label} frame={message.header.frame_id}',
                '  XYZ m:     ' + '  '.join(f'{v:+.5f}' for v in xyz),
                '  q xyzw:    ' + '  '.join(f'{v:+.5f}' for v in q),
                '  RPY deg:   ' + '  '.join(f'{v:+.2f}' for v in rpy)]
    except ValueError as error:
        return [f'{label}: {error}']


def terminal(screen, node):
    screen.nodelay(True)
    screen.keypad(True)
    try:
        curses.curs_set(0)
    except curses.error:
        pass
    next_draw = 0.0
    terminal_events = select.poll()
    terminal_events.register(sys.stdin, select.POLLHUP | select.POLLERR | select.POLLNVAL)
    while rclpy.ok() and not node.done:
        if terminal_events.poll(0):
            node.pause('终端已断开，停止发布')
            break
        rclpy.spin_once(node, timeout_sec=0.005)
        # curses consumes arrow/escape sequences, so their trailing A/B/C/D
        # cannot accidentally be interpreted as movement letter keys.
        for _ in range(16):
            code = screen.getch()
            if code == -1:
                break
            if 0 <= code < 256:
                node.handle_key(chr(code))
            if node.done:
                break
        now = time.monotonic()
        if now < next_draw:
            continue
        next_draw = now + 0.1
        target = node.target if node.armed else node.observed_target
        target_label = 'Target' if node.armed or fresh(target, node.get_clock().now().nanoseconds, .25) else 'Last target (inactive)'
        lines = [f"ServoP | {node.channel['group']} | {'ACTIVE' if node.armed else 'PAUSED'} | "
                 f"axes={'TOOL' if node.local else 'REFERENCE'}",
                 f"topic={node.channel['endpoint']}  tool={node.channel['tip_frame']}",
                 f"step={node.args.step*node.scale*1000:g} mm  angle={node.args.angle_step*node.scale:g} deg  "
                 f"rate={node.args.rate:g} Hz",
                 'Enter: start at FK   Space: pause   C: align to FK   T: reference/tool axes',
                 'W/S: +/-X   A/D: +/-Y   R/F: +/-Z   U/J: +/-Rx   I/K: +/-Ry   O/L: +/-Rz',
                 '+/-: step size   Q/Ctrl+C: quit',
                 'Viewer: cyan ball=actual FK; orange box/ball=target; gray=last target',
                 'Axes: X red, Y green, Z blue. Box extends along target +X.', '',
                 *pose_lines('Actual FK', node.pose), '', *pose_lines(target_label, target)]
        if node.pose is not None and target is not None:
            try:
                if node.pose.header.frame_id == target.header.frame_id:
                    position, angle = pose_error(transform(node.pose), transform(target))
                    lines.append(f'Error: {position*1000:.3f} mm  /  {math.degrees(angle):.3f} deg')
                else:
                    lines.append('Error: poses have different reference frames (see viewer world comparison)')
            except ValueError:
                pass
        ready = node.feedback_reason()
        lines.extend(['', node.notice, 'Feedback: ' + (ready or 'fresh')])
        screen.erase()
        height, width = screen.getmaxyx()
        for row, text in enumerate(lines[:height]):
            try:
                screen.addnstr(row, 0, text, max(0, width-1))
            except curses.error:
                pass
        screen.refresh()


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('必须是有限正数')
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default='', help='与仿真相同的 profile YAML')
    parser.add_argument('--robot-id', default='', help='已部署机器人 ID，与仿真保持一致')
    parser.add_argument('--plugin-root', default='')
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--group', help='运动组，如 left_arm')
    selection.add_argument('--channel', help='channel_config 中的 ServoP 通道名')
    parser.add_argument('--list-channels', action='store_true')
    parser.add_argument('--step', type=positive, default=0.005, help='每次按键平移 m，默认 0.005')
    parser.add_argument('--angle-step', type=positive, default=2.0, help='每次按键转角 deg，默认 2')
    parser.add_argument('--rate', type=positive, default=100.0, help='发布频率 Hz，20～1000')
    parser.add_argument('--max-offset', type=positive, default=0.1, help='目标相对实测位置的最大偏差 m')
    parser.add_argument('--max-angle', type=positive, default=60.0, help='目标相对实测姿态的最大偏差 deg，≤180')
    parser.add_argument('--allow-hardware', action='store_true', help='允许没有仿真状态的机器人接口')
    args, ros_args = parser.parse_known_args(argv)
    if not 20 <= args.rate <= 1000 or args.max_angle > 180:
        parser.error('rate 必须为 20～1000 Hz，max-angle 必须 ≤180°')
    try:
        configuration = load_configuration(get_package_share_directory('humanoid_motion_server'),
                                           args.profile, args.robot_id, args.plugin_root)
        if args.list_channels:
            for item in configuration['channels']:
                if item['kind'] == 'servo_p':
                    print(f"{item['name']}: group={item['group']} topic={item['endpoint']} "
                          f"FK={item.get('fk_pose_topic', '(not configured)')} frame={item['base_frame']}")
            return 0
        channel = select_channel(configuration['channels'], group=args.group, name=args.channel)
    except (ValueError, OSError, KeyError) as error:
        parser.error(str(error))
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        parser.error('请在交互终端运行键盘控制（需要 TTY）')
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 0
    try:
        node = KeyboardServoNode(configuration, channel, args)
        curses.wrapper(terminal, node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(str(error), file=sys.stderr)
        code = 1
    finally:
        if node is not None:
            node.pause()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print('已停止刷新 ServoP 目标')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
