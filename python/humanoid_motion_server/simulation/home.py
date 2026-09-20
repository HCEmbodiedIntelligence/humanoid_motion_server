"""Return an offline robot to its startup pose through the normal MoveJ API."""

import argparse
from collections import deque
import json
import time

from .configuration import read_yaml


HOME_POSE_ID = 'simulation_initial'


def home_targets(configuration):
    """Find disjoint MoveJ groups covering every simulated motion joint."""
    initial, motion = configuration['initial'], configuration['motion']
    candidates = []
    seen = set()
    for channel in configuration['channels']:
        if channel['kind'] != 'move_j':
            continue
        names = motion[f"groups.{channel['group']}"]
        key = frozenset(names)
        if key and key <= initial.keys() and key not in seen:
            seen.add(key)
            candidates.append((key, dict(channel, joint_names=names,
                                         positions=[float(initial[n]) for n in names])))
    candidates.sort(key=lambda item: -len(item[0]))

    def cover(remaining):
        if not remaining:
            return []
        joint = min(remaining)
        for names, target in candidates:
            if joint in names and names <= remaining:
                rest = cover(remaining - names)
                if rest is not None:
                    return [target] + rest
        return None

    result = cover(set(initial))
    if result is None:
        raise ValueError('Simulation home requires disjoint MoveJ channels covering all motion joints')
    return result


def main(args=None):
    import rclpy
    from action_msgs.msg import GoalStatus
    from humanoid_motion_interfaces.action import MoveJ
    from humanoid_motion_interfaces.msg import Status
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool

    parser = argparse.ArgumentParser()
    parser.add_argument('--snapshot', required=True)
    options, ros_args = parser.parse_known_args(args)
    configuration = read_yaml(options.snapshot)
    settings = configuration['simulation_home']

    class HomeNode(Node):
        def __init__(self):
            super().__init__('simulation_home')
            self.phase = 'idle'
            self.request = None
            self.seen = deque(maxlen=128)
            self.frontend_at = self.simulation_at = float('-inf')
            self.targets = home_targets(configuration)
            self.move_clients = [ActionClient(self, MoveJ, t['endpoint']) for t in self.targets]
            self.ownership = self.create_client(SetBool, '/hc_teleop_recv/set_motion_active')
            self.events = self.create_publisher(String, '/hc_teleop_recv/events', 10)
            self.create_subscription(String, '/hc_teleop_recv/actions', self.action, 10)
            self.create_subscription(String, '/hc_teleop_recv/status', self.frontend, 10)
            self.create_subscription(String, '/simulation/status', self.simulation, 10)
            self.create_subscription(Bool, settings['stop_topic'],
                                     lambda msg: self.cancel('回位已取消') if msg.data else None, 10)
            self.create_timer(.02, self.tick)
            self.get_logger().info('Simulation home enabled: both sticks outward; target is startup pose')

        def frontend(self, message):
            try:
                identity = json.loads(message.data)['configuration']
                if identity['sha256'] == settings['configuration_sha256']:
                    self.frontend_at = time.monotonic()
            except (ValueError, KeyError, TypeError):
                pass

        def simulation(self, message):
            try:
                status = json.loads(message.data)
                if (status['mode'] == 'kinematic_simulation' and
                        status['robot_id'] == configuration['robot_id']):
                    self.simulation_at = time.monotonic()
            except (ValueError, KeyError, TypeError):
                pass

        def fresh(self):
            now = time.monotonic()
            return now - self.frontend_at < 1.0 and now - self.simulation_at < 2.5

        def event(self, request, ok, message):
            self.events.publish(String(data=json.dumps({
                'id': request['id'], 'robot_id': settings['robot_id'], 'action': request['action'],
                'kind': 'teleop_action', 'ok': ok, 'message': message})))
            self.get_logger().info(message)

        def action(self, message):
            if len(message.data) > 8192:
                return
            try:
                request = json.loads(message.data)
                valid = (isinstance(request['id'], str) and len(request['id']) == 32 and
                         request['id'] not in self.seen and request['robot_id'] == settings['robot_id'] and
                         request['configuration_sha256'] == settings['configuration_sha256'] and
                         type(request['stamp_ns']) is int and abs(time.time_ns() - request['stamp_ns']) < 2e9)
            except (ValueError, KeyError, TypeError):
                return
            if not valid:
                return
            self.seen.append(request['id'])
            if request.get('action') == 'pause':
                self.cancel('手柄暂停，回位已取消')
                return
            if request.get('action') != 'home':
                return
            if request.get('pose_id') != HOME_POSE_ID:
                self.event(request, False, '仿真仅支持返回本次启动姿态')
                return
            if self.phase != 'idle':
                self.event(request, False, '回位仍在执行或等待停止确认')
                return
            if not self.fresh() or not self.ownership.service_is_ready() or not all(
                    client.server_is_ready() for client in self.move_clients):
                self.event(request, False, '仿真、遥操或 MoveJ 接口尚未就绪，请稍后重试')
                return
            self.request, self.error, self.pending = request, '', []
            self.service = self.ownership.call_async(SetBool.Request(data=True))
            self.phase, self.deadline = 'acquire', time.monotonic() + 3

        def cancel(self, message):
            if self.phase not in ('idle', 'blocked') and not self.error:
                self.error = message

        def finish(self, uncertain=False):
            message = self.error or '已返回仿真初始姿态；按 A 恢复遥操，再握住右握持键'
            if uncertain:
                message += '；未确认控制权释放，遥操保持锁定，请重启仿真'
            self.event(self.request, not self.error and not uncertain, message)
            self.phase = 'blocked' if uncertain else 'idle'

        def release(self):
            self.service = self.ownership.call_async(SetBool.Request(data=False))
            self.phase, self.deadline = 'release', time.monotonic() + 3

        def collect(self):
            for item in self.pending:
                future = item['accept']
                if future.done() and 'handle' not in item:
                    if future.exception():
                        self.cancel('MoveJ 接收结果异常')
                        continue  # Acceptance uncertain: keep ownership until timeout.
                    item['handle'] = future.result()
                    if item['handle'].accepted:
                        item['result'] = item['handle'].get_result_async()
                    else:
                        self.cancel('MoveJ 拒绝回位目标')
                result = item.get('result')
                if result is not None and result.done():
                    if (result.exception() or result.result().status != GoalStatus.STATUS_SUCCEEDED or
                            result.result().result.status.code != Status.OK):
                        self.cancel('回位未完成，已请求停止其余关节组')

        def stopped(self):
            return all('handle' in item and (not item['handle'].accepted or
                       (item['result'].done() and not item['result'].exception())) for item in self.pending)

        def tick(self):
            if self.phase in ('idle', 'blocked'):
                return
            now = time.monotonic()
            if not self.fresh():
                self.cancel('仿真或遥操状态超时，取消回位')
            if self.phase in ('acquire', 'release'):
                if not self.service.done():
                    if now > self.deadline:
                        self.cancel('等待遥操控制权接口超时')
                        self.finish(uncertain=True)
                    return
                if self.service.exception():
                    self.cancel('遥操控制权接口异常')
                    self.finish(uncertain=True)
                    return
                if not self.service.result().success:
                    self.cancel(self.service.result().message)
                    self.finish(uncertain=self.phase == 'release')
                    return
                if self.phase == 'release':
                    self.finish()
                elif self.error:
                    self.release()
                else:
                    # Frontend has stopped publishing Servo. Let its higher
                    # priority lease expire before submitting lower priority MoveJ.
                    self.phase = 'lease'
                    self.deadline = now + configuration['motion'].get('servo_lease_ms', 100) / 1000 + .1
                return
            if self.phase == 'lease':
                if self.error:
                    self.release()
                elif now >= self.deadline:
                    for target, client in zip(self.targets, self.move_clients):
                        goal = MoveJ.Goal()
                        goal.group_name = target['group']
                        goal.target.name = target['joint_names']
                        goal.target.position = target['positions']
                        goal.options.velocity_scale = .25
                        goal.options.acceleration_scale = .25
                        goal.options.jerk_scale = .25
                        goal.options.timeout_sec = 60.0
                        self.pending.append({'accept': client.send_goal_async(goal)})
                    self.phase, self.deadline = 'moving', now + 65
                    self.accept_deadline = now + 3
                return
            self.collect()
            if self.phase == 'moving':
                if now > self.deadline or (now > self.accept_deadline and any(
                        'handle' not in item for item in self.pending)):
                    self.cancel('回位执行或接收目标超时')
                if self.error:
                    self.phase, self.deadline = 'canceling', now + 5
                elif self.stopped():
                    self.release()
            if self.phase == 'canceling':
                # Late accepted goals also have to stop before releasing teleop.
                for item in self.pending:
                    result = item.get('result')
                    if result is not None and not result.done() and not item.get('canceled'):
                        item['handle'].cancel_goal_async()
                        item['canceled'] = True
                if self.stopped():
                    self.release()
                elif now > self.deadline:
                    self.finish(uncertain=True)

    rclpy.init(args=ros_args)
    node = HomeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
