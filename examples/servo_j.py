#!/usr/bin/env python3
"""ServoJ：连续发布关节目标，由运动服务执行已有 RTC 平滑。"""

from sensor_msgs.msg import JointState

from _common import SERVO_QOS, parser, run


def demo(node, args):
    publisher = node.create_publisher(JointState, f'/teleop/{node.group}/servo_j', SERVO_QOS)
    node.wait(lambda: publisher.get_subscription_count() > 0)
    _, positions = node.joint_target(args)
    target = JointState()
    target.name = node.names
    target.position = positions
    # 直接发送目标，RTC 平滑由服务端完成；客户端不做五次插值。
    # 实时关节遥操可在 make_message 中读取最新目标关节角。
    print(f'ServoJ {node.group}: {positions} rad, {args.rate} Hz')
    node.stream(publisher, lambda fraction: target, args)


if __name__ == '__main__':
    args = parser(__doc__, joint=True, stream=True).parse_args()
    raise SystemExit(run('demo_servo_j', args, demo))
