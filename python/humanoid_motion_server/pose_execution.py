"""Shared MoveJ pose execution and teleop ownership for real and simulated robots."""
import time
import uuid
import threading


class MotionCommandError(RuntimeError):
    pass


def execute_pose_with_teleop(goals, domain_id, cancel_event=None, *, receiver_present=True,
                            resume_after=False, servo_lease_ms=100,
                            stop_topic='/teleop/emergency_stop', control=None, execute=None):
    """Own teleop through completion/cancellation; uncertain motion stays inhibited.

    The independent runtime checks feedback and provides a cancellation event.
    Hardware and simulation use the same execution and resume policy.
    """
    control = control or receiver_control
    execute = execute or execute_move_j_pose
    cancel_event = cancel_event or threading.Event()
    acquired = completed = False
    try:
        if cancel_event.is_set():
            raise MotionCommandError('回位已取消')
        if receiver_present:
            control('acquire', domain_id)
            acquired = True
            cancel_event.wait(float(servo_lease_ms) / 1000 + .05)
        if cancel_event.is_set():
            raise MotionCommandError('回位已取消')
        try:
            results = execute(goals, domain_id, cancel_event, stop_topic)
        except Exception as error:
            if getattr(error, 'motion_may_be_active', not isinstance(error, MotionCommandError)):
                acquired = False
            raise
        completed = True
        return results
    finally:
        if acquired:
            try:
                control('release', domain_id)
            except Exception as error:
                failure = MotionCommandError('未确认遥操控制权释放：' + str(error))
                failure.motion_may_be_active = True
                raise failure from error
            if completed and resume_after and not cancel_event.is_set():
                control('resume', domain_id)


def _spin_until(executor, futures, deadline):
    while not all(future.done() for future in futures):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        executor.spin_once(timeout_sec=min(0.05, remaining))
    return True


def execute_move_j_pose(goals, domain_id, cancel_event=None, stop_topic='/teleop/emergency_stop'):
    """Execute disjoint joint goals; do not release ownership until all stop."""
    import threading
    if not goals:
        raise MotionCommandError('初始姿态没有可执行目标')
    cancel_event = cancel_event or threading.Event()
    import rclpy
    from action_msgs.msg import GoalStatus
    from humanoid_motion_interfaces.action import MoveJ
    from humanoid_motion_interfaces.msg import Status
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Bool
    context = Context()
    node = executor = None
    clients, accepted, results, acceptance = [], [], [], []
    try:
        rclpy.init(args=[], context=context, domain_id=int(domain_id))
        node = rclpy.create_node('humanoid_pose_' + uuid.uuid4().hex[:10], context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        stop_sub = node.create_subscription(Bool, stop_topic,
            lambda msg: cancel_event.set() if msg.data else None, 10)
        prepared = []
        for target in goals:
            if cancel_event.is_set():
                raise MotionCommandError('回位已取消')
            client = ActionClient(node, MoveJ, target['endpoint'])
            clients.append(client)
            if not client.wait_for_server(timeout_sec=2):
                raise MotionCommandError('MoveJ 接口不可用: ' + target['endpoint'])
            goal = MoveJ.Goal()
            goal.group_name = target['group']
            goal.target.name = list(target['joint_names'])
            goal.target.position = [float(v) for v in target['positions_rad']]
            for key in ('velocity_scale', 'acceleration_scale', 'jerk_scale', 'timeout_sec'):
                setattr(goal.options, key, float(target[key]))
            prepared.append((target, client, goal))
        if cancel_event.is_set():
            raise MotionCommandError('回位已取消')
        for _, client, goal in prepared:
            acceptance.append(client.send_goal_async(goal))
        complete = _spin_until(executor, acceptance, time.monotonic() + 5)
        errors = []
        for (target, _, _), future in zip(prepared, acceptance):
            if not future.done():
                continue
            if future.exception() is not None:
                errors.append(str(future.exception()))
                continue
            handle = future.result()
            if handle.accepted:
                accepted.append((target, handle))
                results.append(handle.get_result_async())
            else:
                errors.append('MoveJ 拒绝目标: ' + target['endpoint'])
        if not complete:
            raise MotionCommandError('等待 MoveJ 接收目标超时')
        if errors:
            raise MotionCommandError('; '.join(errors))
        deadline = time.monotonic() + max(float(g['timeout_sec']) for g in goals) + 10
        while not all(f.done() for f in results):
            if cancel_event.is_set():
                raise MotionCommandError('回位已取消')
            if time.monotonic() >= deadline:
                raise MotionCommandError('回位执行超时')
            executor.spin_once(timeout_sec=.05)
            for future in results:
                if future.done() and (future.exception() or future.result().status != GoalStatus.STATUS_SUCCEEDED
                                      or future.result().result.status.code != Status.OK):
                    raise MotionCommandError('部分关节组回位失败，已请求取消其余目标')
        output = []
        for (target, _), future in zip(accepted, results):
            wrapped = future.result()
            status = wrapped.result.status
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or status.code != Status.OK:
                raise MotionCommandError(target['group'] + ' 未到达初始姿态: ' + status.message)
            state = wrapped.result.final_joint_state
            output.append({'channel': target['channel'], 'endpoint': target['endpoint'],
                'group': target['group'], 'goal_status': int(wrapped.status),
                'status_code': int(status.code), 'message': status.message,
                'final_joint_state': {'name': list(state.name), 'position': list(state.position)}})
        return output
    except Exception as original:
        # Include every accepted goal, including goals accepted after another was rejected.
        uncertain = False
        try:
            processed = {bytes(handle.goal_id.uuid) for _, handle in accepted}
            canceled = set()
            deadline = time.monotonic() + 5
            while acceptance:
                # A late acceptance still owns motion, even after another goal failed.
                for (target, _, _), future in zip(prepared, acceptance):
                    if future.done() and not future.exception():
                        handle = future.result()
                        key = bytes(handle.goal_id.uuid)
                        if handle.accepted and key not in processed:
                            processed.add(key)
                            accepted.append((target, handle))
                            results.append(handle.get_result_async())
                for (_, handle), result in zip(accepted, results):
                    key = bytes(handle.goal_id.uuid)
                    if not result.done() and key not in canceled:
                        canceled.add(key)
                        try:
                            handle.cancel_goal_async()
                        except Exception:
                            uncertain = True
                if all(f.done() for f in acceptance + results) or time.monotonic() >= deadline:
                    break
                executor.spin_once(timeout_sec=.05)
            uncertain |= any(not f.done() or f.exception() for f in acceptance + results)
        except Exception:
            uncertain = True
        error = original if isinstance(original, MotionCommandError) else MotionCommandError(str(original))
        error.motion_may_be_active = uncertain
        if uncertain:
            error.args = (str(error) + '；未确认全部目标停止，遥操保持暂停，请检查运动服务',)
        raise error
    finally:
        if executor:
            executor.shutdown()
        for client in clients:
            client.destroy()
        if node:
            node.destroy_node()
        if context.ok():
            context.shutdown()


def receiver_control(action, domain_id):
    """A short-lived service client; never publishes a motion target."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import SetBool, Trigger
    choices = {'pause': ('set_enabled', False), 'resume': ('set_enabled', True),
               'acquire': ('set_motion_active', True), 'release': ('set_motion_active', False),
               'reset_reference': ('reset_reference', None)}
    service, value = choices[action]
    kind = Trigger if value is None else SetBool
    context = Context()
    node = executor = client = None
    try:
        rclpy.init(args=[], context=context, domain_id=int(domain_id))
        node = rclpy.create_node('teleop_control_client', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        client = node.create_client(kind, '/hc_teleop_recv/' + service)
        if not client.wait_for_service(timeout_sec=2):
            raise MotionCommandError('遥操控制接口未就绪，请确认接收节点已更新并启动')
        request = kind.Request()
        if value is not None:
            request.data = value
        future = client.call_async(request)
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        if not future.done():
            raise MotionCommandError('遥操控制请求超时，状态未知，请查看遥操状态')
        response = future.result()
        if not response.success:
            raise MotionCommandError(response.message)
        return response.message
    finally:
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
        if context.ok():
            context.shutdown()
