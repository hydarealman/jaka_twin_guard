"""Offline regression tests: never connect to a controller."""
from concurrent.futures import Future
from types import SimpleNamespace, MethodType
import threading

from fruit_picking_arm.planner.planner_server import SingleArmPlannerServer


def planner_shell():
    p = SimpleNamespace(
        _command_lock=threading.RLock(), _cancel_epoch=0,
        _motion_latched=False, _motion_guard=lambda: True,
        _pending_goals=set(), _tracked_goals={},
        _spin_both=lambda future, timeout_sec: None,
    )
    for name in ("_send_guarded_goal", "_forget_goal", "_await_result",
                 "cancel_active_goal", "arm_motion"):
        setattr(p, name, MethodType(getattr(SingleArmPlannerServer, name), p))
    return p


class Handle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancels = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancels += 1


def test_stop_during_planning_prevents_any_submission():
    p = planner_shell()
    p.cancel_active_goal()
    client = SimpleNamespace(send_goal_async=lambda _: (_ for _ in ()).throw(
        AssertionError("motion submitted after stop")))
    assert p._send_guarded_goal(client, object(), 1) is None


def test_late_goal_acceptance_is_cancelled_and_blocks_rearm_until_terminal():
    p = planner_shell()
    accepted = Future()
    client = SimpleNamespace(send_goal_async=lambda _: accepted)
    assert p._send_guarded_goal(client, object(), 1) is None
    assert not p.arm_motion()
    handle = Handle()
    accepted.set_result(handle)
    assert handle.cancels > 0
    assert not p.arm_motion()
    handle.result.set_result(SimpleNamespace(status=5))
    assert p.arm_motion()


def test_result_timeout_cancels_and_blocks_rearm():
    p = planner_shell()
    handle = Handle()
    accepted = Future()
    accepted.set_result(handle)
    client = SimpleNamespace(send_goal_async=lambda _: accepted)
    assert p._send_guarded_goal(client, object(), 1) is handle
    assert p._await_result(handle, 1) is None
    assert handle.cancels > 0
    assert not p.arm_motion()


def test_stop_cancels_both_arm_and_gripper():
    p = planner_shell()
    handles = [Handle(), Handle()]
    for handle in handles:
        accepted = Future()
        accepted.set_result(handle)
        p._send_guarded_goal(SimpleNamespace(send_goal_async=lambda _: accepted), object(), 1)
    p.cancel_active_goal()
    assert all(handle.cancels == 1 for handle in handles)


def test_result_transport_error_is_not_proof_of_stopped_motors():
    p = planner_shell()
    handle = Handle()
    accepted = Future()
    accepted.set_result(handle)
    p._send_guarded_goal(SimpleNamespace(send_goal_async=lambda _: accepted), object(), 1)
    handle.result.set_exception(RuntimeError("transport lost"))
    assert not p.arm_motion()
    assert handle.cancels > 0
