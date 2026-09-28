#!/usr/bin/env python3
"""MoveJ：一次提交关节目标，等待 Action 终态和业务状态码。"""

from humanoid_motion_interfaces.action import MoveJ

from _common import parser, run


def demo(node, args):
    client = node.action_client(MoveJ, 'move_j')
    try:
        _, positions = node.joint_target(args)
        goal = MoveJ.Goal()
        goal.group_name = node.group
        goal.target.header.stamp = node.get_clock().now().to_msg()
        goal.target.name = node.names
        goal.target.position = positions
        print(f'MoveJ {node.group}: {positions} rad')
        node.execute(client, goal, args)
    finally:
        client.destroy()


if __name__ == '__main__':
    args = parser(__doc__, joint=True).parse_args()
    raise SystemExit(run('demo_move_j', args, demo))
