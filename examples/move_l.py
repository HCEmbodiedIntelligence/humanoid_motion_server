#!/usr/bin/env python3
"""MoveL：从当前 FK 沿指定方向移动，保持姿态并等待结果。"""

from humanoid_motion_interfaces.action import MoveL

from _common import parser, run


def demo(node, args):
    client = node.action_client(MoveL, 'move_l')
    try:
        goal = MoveL.Goal()
        goal.group_name = node.group
        goal.tip_frame = args.arm + '_tool0'
        goal.target_pose = node.pose_target(args)
        goal.target_pose.header.stamp = node.get_clock().now().to_msg()
        print(f'MoveL {node.group}: frame={goal.target_pose.header.frame_id}, '
              f'offset=({args.dx}, {args.dy}, {args.dz}) m')
        node.execute(client, goal, args)
    finally:
        client.destroy()


if __name__ == '__main__':
    args = parser(__doc__).parse_args()
    raise SystemExit(run('demo_move_l', args, demo))
