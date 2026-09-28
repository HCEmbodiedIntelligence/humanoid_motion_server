#!/usr/bin/env python3
"""ServoP：以当前 FK 加一次偏移为目标，持续发布末端位姿。"""

from geometry_msgs.msg import PoseStamped

from _common import SERVO_QOS, parser, run


def demo(node, args):
    publisher = node.create_publisher(PoseStamped, f'/teleop/{node.group}/servo_p', SERVO_QOS)
    node.wait(lambda: publisher.get_subscription_count() > 0)
    node.wait(node.valid_state)
    target = node.pose_target(args)
    # 固定目标只计算一次；每帧刷新时间戳，不在循环中累加 dx/dy/dz。
    # 实时遥操可在 make_message 中读取最新手柄目标。
    print(f'ServoP {node.group}: frame={target.header.frame_id}, {args.rate} Hz')
    node.stream(publisher, lambda fraction: target, args, require_pose=True)


if __name__ == '__main__':
    args = parser(__doc__, stream=True).parse_args()
    raise SystemExit(run('demo_servo_p', args, demo))
