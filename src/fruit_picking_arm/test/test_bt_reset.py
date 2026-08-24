from fruit_picking_arm.behavior.bt_node_base import (
    BtActionNode,
    NodeStatus,
    RetryNode,
    Sequence,
)


class _CountingSuccess(BtActionNode):
    def __init__(self):
        super().__init__("counting_success")
        self.calls = 0

    def execute(self):
        self.calls += 1
        return NodeStatus.SUCCESS


def test_sequence_reset_recursively_reexecutes_children():
    first = _CountingSuccess()
    second = _CountingSuccess()
    root = Sequence("root", [first, second])

    assert root.tick() == NodeStatus.SUCCESS
    assert (first.calls, second.calls) == (1, 1)

    root.reset()

    assert root.status == NodeStatus.IDLE
    assert first.status == NodeStatus.IDLE
    assert second.status == NodeStatus.IDLE
    assert root.tick() == NodeStatus.SUCCESS
    assert (first.calls, second.calls) == (2, 2)


def test_retry_reset_recursively_reexecutes_child():
    child = _CountingSuccess()
    root = RetryNode("retry", child, max_retries=3)

    assert root.tick() == NodeStatus.SUCCESS
    root.reset()
    assert child.status == NodeStatus.IDLE
    assert root.tick() == NodeStatus.SUCCESS
    assert child.calls == 2

