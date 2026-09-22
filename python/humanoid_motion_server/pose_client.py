"""Request/status transport only; all motion decisions live in pose_runtime."""
import json
import time
import uuid

from .pose_execution import MotionCommandError


def request_pose_action(action, robot_id, revision, domain_id, pose_id=None, *, request_stamp_ns=None,
                        joint_name=None, delta_rad=None):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    context = Context()
    node = executor = None
    ident = uuid.uuid4().hex
    state = {}
    result = None
    request = {'id': ident, 'action': action, 'robot_id': robot_id, 'revision': revision}
    if pose_id is not None:
        request['pose_id'] = pose_id
    if action == 'jog_joint':
        request.update(joint_name=joint_name, delta_rad=delta_rad)

    def status(message):
        nonlocal state
        try:
            value = json.loads(message.data)
            if (isinstance(value, dict) and value.get('robot_id') == robot_id and value.get('revision') == revision and
                    0 <= time.time_ns() - value['stamp_ns'] < 1_000_000_000):
                state = value
        except (ValueError, KeyError, TypeError):
            pass

    def receive(message):
        nonlocal result
        try:
            value = json.loads(message.data)
            if isinstance(value, dict) and value.get('id') == ident and value.get('robot_id') == robot_id and value.get('revision') == revision:
                result = value
        except (ValueError, TypeError):
            pass

    try:
        rclpy.init(args=[], context=context, domain_id=int(domain_id))
        node = rclpy.create_node('pose_request_' + ident[:10], context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        subscriptions = [node.create_subscription(String, '/motion/pose_status', status, 10),
                         node.create_subscription(String, '/motion/pose_results', receive, 10)]
        publisher = node.create_publisher(String, '/motion/pose_requests', 10)
        deadline = time.monotonic() + 3
        while not state or not publisher.get_subscription_count():
            if time.monotonic() >= deadline:
                raise MotionCommandError('对应机器人版本的姿态运行时未就绪')
            executor.spin_once(timeout_sec=.05)
        request['stamp_ns'] = request_stamp_ns or time.time_ns()
        publisher.publish(String(data=json.dumps(request)))
        timeout = max((p['timeout_sec'] for p in state.get('poses', [])), default=60) + 20
        deadline = time.monotonic() + (timeout if action in {'home', 'execute_pose', 'jog_joint'} else 10)
        while result is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        if result is None:
            raise MotionCommandError('等待运行时结果超时，请查看实际运动状态；网页未取消运行中的请求')
        if not result.get('ok'):
            error = MotionCommandError(result.get('message', '运行时拒绝操作'))
            error.motion_may_be_active = result.get('motion_may_be_active', False)
            raise error
        return result
    finally:
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
        if context.ok():
            context.shutdown()
