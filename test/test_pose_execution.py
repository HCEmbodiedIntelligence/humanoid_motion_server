"""Shared lifecycle behavior, including cancellation and uncertain ownership."""
import threading

import pytest

from humanoid_motion_server.pose_execution import MotionCommandError, execute_pose_with_teleop


@pytest.mark.parametrize('mode,expected', [
    ('success', ['acquire', 'move', 'release', 'resume']),
    ('disabled', ['acquire', 'move', 'release']),
    ('failure', ['acquire', 'move', 'release']),
    ('uncertain', ['acquire', 'move']),
    ('cancel_before', []),
    ('cancel_acquire', ['acquire', 'release']),
    ('cancel_move', ['acquire', 'move', 'release']),
    ('release_failure', ['acquire', 'move', 'release']),
])
def test_pose_ownership_lifecycle(mode, expected):
    cancel = threading.Event()
    calls = []
    if mode == 'cancel_before':
        cancel.set()

    def control(action, domain):
        calls.append(action)
        if action == 'acquire' and mode == 'cancel_acquire':
            cancel.set()
        if action == 'release' and mode == 'release_failure':
            raise MotionCommandError('release timeout')

    def execute(goals, domain, event, stop):
        calls.append('move')
        assert event is cancel and goals == ['pose']
        if mode in {'failure', 'uncertain', 'cancel_move'}:
            if mode == 'cancel_move':
                event.set()
            error = MotionCommandError(mode)
            error.motion_may_be_active = mode == 'uncertain'
            raise error
        return ['done']

    if mode in {'success', 'disabled'}:
        assert execute_pose_with_teleop(['pose'], 230, cancel, resume_after=mode != 'disabled',
                                       servo_lease_ms=0, control=control, execute=execute) == ['done']
    else:
        with pytest.raises(MotionCommandError):
            execute_pose_with_teleop(['pose'], 230, cancel, resume_after=True,
                                    servo_lease_ms=0, control=control, execute=execute)
    assert calls == expected


def test_pose_without_receiver_uses_the_same_executor():
    calls = []
    execute_pose_with_teleop([], 230, receiver_present=False,
                            control=lambda *a: pytest.fail('no receiver configured'),
                            execute=lambda *a: calls.append('move'))
    assert calls == ['move']
