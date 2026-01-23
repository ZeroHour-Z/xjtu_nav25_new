from typing import Any

import py_trees
from py_trees.common import Status
from rclpy.node import Node

from ..registry import register


@register("Wait")
class Wait(py_trees.behaviour.Behaviour):
	def __init__(self, name: str, node: Node, duration_s: float = 1.0):
		super().__init__(name)
		self.node = node
		self.duration_s = duration_s
		self._start_time = None

	def initialise(self) -> None:
		self._start_time = self.node.get_clock().now()

	def update(self) -> Status:
		delta = (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9
		return Status.SUCCESS if delta >= self.duration_s else Status.RUNNING


@register("SetBlackboard")
class SetBlackboard(py_trees.behaviour.Behaviour):
	def __init__(self, name: str, node: Node, key: str, value: Any):
		super().__init__(name)
		self.node = node
		self.key = key
		self.value = value
		self.blackboard = py_trees.blackboard.Client(name=f"bb_{name}")
		self.blackboard.register_key(key=self.key, access=py_trees.common.Access.WRITE)

	def update(self) -> Status:
		setattr(self.blackboard, self.key, self.value)
		return Status.SUCCESS


@register("CheckBlackboard")
class CheckBlackboard(py_trees.behaviour.Behaviour):
	def __init__(self, name: str, node: Node, key: str, expected: Any = True):
		super().__init__(name)
		self.node = node
		self.key = key
		self.expected = expected
		self.blackboard = py_trees.blackboard.Client(name=f"bb_{name}")
		self.blackboard.register_key(key=self.key, access=py_trees.common.Access.READ)

	def update(self) -> Status:
		try:
			current_value = getattr(self.blackboard, self.key)
		except Exception:
			return Status.FAILURE
		return Status.SUCCESS if current_value == self.expected else Status.FAILURE


@register("Success")
class Success(py_trees.behaviour.Behaviour):
	def __init__(self, name: str, node: Node):
		super().__init__(name)
		self.node = node

	def update(self) -> Status:
		return Status.SUCCESS 