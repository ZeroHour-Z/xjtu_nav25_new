from typing import Optional

import py_trees
from py_trees.common import Status
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from ..registry import register


@register("ChaseDynamicPointAction")
class ChaseDynamicPointAction(py_trees.behaviour.Behaviour):
	def __init__(
		self,
		name: str,
		node: Node,
		topic: str = "/chase_point",
		server_name: str = "navigate_to_pose",
		cancel_on_terminate: bool = True,
	):
		super().__init__(name)
		self.node = node
		self.topic = topic
		self.client = ActionClient(node, NavigateToPose, server_name)
		self.cancel_on_terminate = cancel_on_terminate
		self._goal_handle = None
		self._result_future = None
		self._last_pose: Optional[PoseStamped] = None
		self._has_new_target: bool = False
		self._sub = None

	def setup(self, **kwargs) -> None:
		if self._sub is None:
			self._sub = self.node.create_subscription(PoseStamped, self.topic, self._on_target, 10)
		# wait briefly for server
		timeout_sec = float(kwargs.get('timeout', 3.0)) if 'timeout' in kwargs else 3.0
		if not self.client.wait_for_server(timeout_sec=timeout_sec):
			raise RuntimeError("Nav2 NavigateToPose action server not available")

	def initialise(self) -> None:
		self._goal_handle = None
		self._result_future = None
		self._has_new_target = False

	def update(self) -> Status:
		# Must have a target before we can chase
		if self._last_pose is None:
			return Status.RUNNING

		# If we got a new target, (re)send goal
		if self._has_new_target:
			# Cancel previous goal if active
			if self._goal_handle is not None:
				try:
					self._goal_handle.cancel_goal_async()
				except Exception as exc:
					self.node.get_logger().warn(f"Cancel previous chase goal failed: {exc}")
			# Send new goal
			goal_msg = NavigateToPose.Goal()
			goal_msg.pose = self._last_pose
			send_future = self.client.send_goal_async(goal_msg)
			send_future.add_done_callback(self._on_goal_response)
			self._has_new_target = False
			return Status.RUNNING

		# If current goal finished, we still keep running to accept new targets
		if self._result_future is not None and self._result_future.done():
			# Ignore result status to keep chasing loop alive
			return Status.RUNNING

		return Status.RUNNING

	def terminate(self, new_status: Status) -> None:
		if new_status == Status.INVALID and self.cancel_on_terminate and self._goal_handle is not None:
			try:
				self._goal_handle.cancel_goal_async()
			except Exception as exc:
				self.node.get_logger().warn(f"Cancel chase goal failed on terminate: {exc}")

	def _on_target(self, pose: PoseStamped) -> None:
		self._last_pose = pose
		self._has_new_target = True

	def _on_goal_response(self, future):
		self._goal_handle = future.result()
		if not getattr(self._goal_handle, 'accepted', False):
			self._result_future = None
			self.node.get_logger().warn(f"Chase goal rejected")
		else:
			self._result_future = self._goal_handle.get_result_async() 