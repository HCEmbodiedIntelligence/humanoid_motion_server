"""Shared ROS setup, fresh feedback and action handling for the motion demos."""

import argparse
import copy
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState


SERVO_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
COMMAND_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


def finite(value):
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError('必须为有限数值')
    return result


def positive(value):
    result = finite(value)
    if result <= 0:
        raise argparse.ArgumentTypeError('必须大于 0')
    return result


def scale(value):
    result = positive(value)
    if result > 1:
        raise argparse.ArgumentTypeError('比例必须在 (0, 1] 内')
    return result


def rate(value):
    result = positive(value)
    if not 20 <= result <= 1000:
        raise argparse.ArgumentTypeError('频率必须在 20～1000 Hz 内')
    return result


def parser(description, *, joint=False, stream=False):
    result = argparse.ArgumentParser(description=description)
    result.add_argument('--arm', choices=('left', 'right'), default='left')
    if joint:
        result.add_argument('--joint', type=int, choices=range(1, 8), default=4)
        target = result.add_mutually_exclusive_group()
        target.add_argument('--delta', type=finite, default=0.0, help='单关节相对位移 rad')
        target.add_argument('--positions', nargs=7, type=finite, help='joint1～7 绝对角度 rad')
    else:
        result.add_argument('--dx', type=finite, default=0.0, help='相对当前 FK 的 X 位移 m')
        result.add_argument('--dy', type=finite, default=0.0, help='相对当前 FK 的 Y 位移 m')
        result.add_argument('--dz', type=finite, default=0.0, help='相对当前 FK 的 Z 位移 m')
    if stream:
        result.add_argument('--rate', type=rate, default=100.0, help='发布频率 Hz')
        result.add_argument('--duration', type=positive, default=3.0, help='持续时间 s')
    else:
        result.add_argument('--scale', type=scale, default=0.1, help='速度、加速度和 jerk 比例')
        result.add_argument('--timeout', type=positive, default=30.0, help='服务端运动超时 s')
    return result


class DemoNode(Node):
    def __init__(self, name, arm):
        super().__init__(name)
        self.arm = arm
        self.group = arm + '_arm'
        self.names = [f'openarmx_{arm}_joint{i}' for i in range(1, 8)]
        self.state = None
        self.pose = None
        self.create_subscription(JointState, '/hc_teleop/joint_states',
                                 lambda msg: setattr(self, 'state', msg), SERVO_QOS)
        self.create_subscription(PoseStamped, f'/teleop/{self.group}/fk_pose',
                                 lambda msg: setattr(self, 'pose', msg), SERVO_QOS)

    def wait(self, condition, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() >= deadline:
                raise RuntimeError('等待 ROS 接口、反馈或结果超时')
            rclpy.spin_once(self, timeout_sec=0.01)

    def fresh(self, message):
        if message is None:
            return False
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return stamp > 0 and -0.1 <= age < 0.1

    def valid_state(self):
        state = self.state
        return (self.fresh(state) and len(state.name) == len(state.position)
                and len(set(state.name)) == len(state.name)
                and all(math.isfinite(x) for x in state.position)
                and set(self.names).issubset(state.name))

    def joint_target(self, args):
        self.wait(self.valid_state)
        measured = dict(zip(self.state.name, self.state.position))
        start = [measured[name] for name in self.names]
        target = list(args.positions) if args.positions is not None else start.copy()
        if args.positions is None:
            target[args.joint - 1] += args.delta
        return start, target

    def pose_target(self, args):
        self.wait(lambda: self.fresh(self.pose))
        target = copy.deepcopy(self.pose)
        p, q = target.pose.position, target.pose.orientation
        if (not target.header.frame_id or
                not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))):
            raise RuntimeError('FK 坐标系或数值无效')
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-12:
            raise RuntimeError('FK 四元数无效')
        q.x, q.y, q.z, q.w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        p.x += args.dx
        p.y += args.dy
        p.z += args.dz
        return target

    def future_result(self, future, timeout):
        self.wait(future.done, timeout)
        return future.result()

    def execute(self, client, goal, args):
        goal.options.velocity_scale = args.scale
        goal.options.acceleration_scale = args.scale
        goal.options.jerk_scale = args.scale
        goal.options.timeout_sec = args.timeout
        pending = client.send_goal_async(goal)
        handle = result = None
        try:
            handle = self.future_result(pending, 5.0)
            if not handle.accepted:
                raise RuntimeError('Action 拒绝 Goal')
            result = handle.get_result_async()
            wrapped = self.future_result(result, args.timeout + 5.0)
            status = wrapped.result.status
            print(f'Action status={wrapped.status}, code={status.code}, message={status.message}')
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or status.code != 0:
                raise RuntimeError('运动未成功完成')
        finally:
            # Keep ROS alive on Ctrl+C so an accepted Goal can be canceled.
            if handle is None or (handle.accepted and (result is None or not result.done())):
                try:
                    handle = handle or self.future_result(pending, 5.0)
                    if handle.accepted:
                        response = self.future_result(handle.cancel_goal_async(), 3.0)
                        print(f'取消响应：{len(response.goals_canceling)} 个 Goal 正在取消')
                        final = self.future_result(result or handle.get_result_async(), 3.0)
                        if final.status not in (GoalStatus.STATUS_CANCELED,
                                                GoalStatus.STATUS_SUCCEEDED,
                                                GoalStatus.STATUS_ABORTED):
                            raise RuntimeError(f'非终态：{final.status}')
                        print(f'已确认 Action 终态：{final.status}')
                except Exception as error:
                    print(f'未确认动作停止：{error}；检查服务端状态', file=sys.stderr)

    def action_client(self, action_type, name):
        client = ActionClient(self, action_type, f'/motion/{self.group}/{name}')
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('Action Server 未就绪')
        return client

    def stream(self, publisher, make_message, args, *, hold=0.0, require_pose=False):
        self.wait(lambda: publisher.get_subscription_count() > 0)
        begin = time.monotonic()
        last_publish = None
        next_tick = begin
        count = 0
        while True:
            now = time.monotonic()
            elapsed = now - begin
            if elapsed > args.duration + hold:
                break
            if now < next_tick:
                rclpy.spin_once(self, timeout_sec=min(0.005, next_tick - now))
                continue
            if last_publish is not None and now - last_publish >= 0.1:
                raise RuntimeError('发布间隔超过 100 ms，停止本次输出')
            if not self.valid_state() or (require_pose and not self.fresh(self.pose)):
                raise RuntimeError('反馈过期，停止发布')
            message = make_message(min(elapsed / args.duration, 1.0))
            message.header.stamp = self.get_clock().now().to_msg()
            publisher.publish(message)
            count += 1
            last_publish = now
            next_tick = now + 1.0 / args.rate
            rclpy.spin_once(self, timeout_sec=0.0)
        print(f'已发布 {count} 帧；停止刷新目标（Topic 无 Action 到位结果）')


def run(name, args, callback):
    # Leave Python's SIGINT handler in place for action cancellation on Ctrl+C.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = DemoNode(name, args.arm)
    code = 0
    try:
        callback(node, args)
    except KeyboardInterrupt:
        print('用户中断，结束示例')
        code = 130
    except Exception as error:
        print(str(error), file=sys.stderr)
        code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code
