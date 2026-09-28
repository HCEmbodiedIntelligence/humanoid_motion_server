#!/usr/bin/env python3
"""joint_cmd：连续关节指令直达驱动；运行前停用其他关节指令发送源。"""

from sensor_msgs.msg import JointState

from _common import COMMAND_QOS, parser, run


def demo(node, args):
    publisher = node.create_publisher(JointState, '/hc_teleop/joint_cmd', COMMAND_QOS)
    node.wait(lambda: publisher.get_subscription_count() > 0)
    start, target = node.joint_target(args)
    delta = [b - a for a, b in zip(start, target)]
    print(f'joint_cmd {node.group}: {target} rad, {args.duration} s, {args.rate} Hz')

    def command(u):
        # 本次 Demo 新增的五次插值，起止速度、加速度为零；并非驱动内置平滑。
        # 上游已有连续指令时可直接逐帧发布；此处不做模型限位检查。
        s = 10*u**3 - 15*u**4 + 6*u**5
        ds_dt = (30*u**2 - 60*u**3 + 30*u**4) / args.duration
        message = JointState()
        message.name = node.names
        message.position = [q + s*d for q, d in zip(start, delta)]
        message.velocity = [ds_dt*d for d in delta]
        return message

    # 到达目标后继续刷新 0.2 s，保证发出最终位置和零速度。
    node.stream(publisher, command, args, hold=0.2)


if __name__ == '__main__':
    args = parser(__doc__, joint=True, stream=True).parse_args()
    raise SystemExit(run('demo_joint_cmd', args, demo))
