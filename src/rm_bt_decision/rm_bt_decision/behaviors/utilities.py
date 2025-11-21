from typing import Any
import math

import py_trees
from py_trees.common import Status
from rclpy.node import Node
from rclpy import time as rclpy_time
from tf2_ros import Buffer, TransformListener

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


@register("RecordStartPosition")
class RecordStartPosition(py_trees.behaviour.Behaviour):
	"""记录当前机器人位置到 Blackboard 作为起始位置"""
	def __init__(self, name: str, node: Node, base_frame: str = "base_link", map_frame: str = "map"):
		super().__init__(name)
		self.node = node
		self.base_frame = base_frame
		self.map_frame = map_frame
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, node)
		self.blackboard = py_trees.blackboard.Client(name=f"bb_{name}")
		self.blackboard.register_key(key="start_x", access=py_trees.common.Access.WRITE)
		self.blackboard.register_key(key="start_y", access=py_trees.common.Access.WRITE)
		self.blackboard.register_key(key="start_yaw", access=py_trees.common.Access.WRITE)

	def update(self) -> Status:
		try:
			transform = self.tf_buffer.lookup_transform(
				self.map_frame, self.base_frame, rclpy_time.Time()
			)
			x = transform.transform.translation.x
			y = transform.transform.translation.y
			q = transform.transform.rotation
			# 计算yaw角
			siny_cosp = 2 * (q.w * q.z + q.x * q.y)
			cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
			yaw = math.atan2(siny_cosp, cosy_cosp)
			
			setattr(self.blackboard, "start_x", x)
			setattr(self.blackboard, "start_y", y)
			setattr(self.blackboard, "start_yaw", yaw)
			
			self.node.get_logger().info(f"Recorded start position: ({x:.2f}, {y:.2f}, {yaw:.2f})")
			return Status.SUCCESS
		except Exception as e:
			self.node.get_logger().warn(f"Failed to record start position: {e}")
			return Status.RUNNING


@register("NavigateToStartPosition")
class NavigateToStartPosition(py_trees.behaviour.Behaviour):
	"""导航到 Blackboard 中记录的起始位置"""
	def __init__(
		self,
		name: str,
		node: Node,
		server_name: str = "navigate_to_pose",
		timeout_s: float = 120.0,
		cancel_on_terminate: bool = True,
	):
		super().__init__(name)
		self.node = node
		self.client = None
		self.server_name = server_name
		self.timeout_s = timeout_s
		self.cancel_on_terminate = cancel_on_terminate
		self._goal_handle = None
		self._result_future = None
		self._sent = False
		self._start_time = None
		self.blackboard = py_trees.blackboard.Client(name=f"bb_{name}")
		self.blackboard.register_key(key="start_x", access=py_trees.common.Access.READ)
		self.blackboard.register_key(key="start_y", access=py_trees.common.Access.READ)
		self.blackboard.register_key(key="start_yaw", access=py_trees.common.Access.READ)

	def setup(self, **kwargs) -> None:
		from rclpy.action import ActionClient
		from nav2_msgs.action import NavigateToPose
		self.client = ActionClient(self.node, NavigateToPose, self.server_name)
		timeout_sec = float(kwargs.get('timeout', 3.0)) if 'timeout' in kwargs else 3.0
		if not self.client.wait_for_server(timeout_sec=timeout_sec):
			raise RuntimeError("Nav2 NavigateToPose action server not available")

	def initialise(self) -> None:
		self._sent = False
		self._goal_handle = None
		self._result_future = None
		self._start_time = self.node.get_clock().now()

	def update(self) -> Status:
		from nav2_msgs.action import NavigateToPose
		from geometry_msgs.msg import PoseStamped
		import math
		
		if not self._sent:
			try:
				start_x = getattr(self.blackboard, "start_x")
				start_y = getattr(self.blackboard, "start_y")
				start_yaw = getattr(self.blackboard, "start_yaw")
			except Exception:
				self.node.get_logger().warn("Start position not recorded in blackboard")
				return Status.FAILURE
			
			goal_msg = NavigateToPose.Goal()
			pose = PoseStamped()
			pose.header.frame_id = "map"
			pose.header.stamp = self.node.get_clock().now().to_msg()
			pose.pose.position.x = float(start_x)
			pose.pose.position.y = float(start_y)
			pose.pose.orientation.z = math.sin(start_yaw * 0.5)
			pose.pose.orientation.w = math.cos(start_yaw * 0.5)
			goal_msg.pose = pose
			
			send_future = self.client.send_goal_async(goal_msg)
			send_future.add_done_callback(self._on_goal_response)
			self._sent = True
			self.node.get_logger().info(f"Navigating to start position: ({start_x:.2f}, {start_y:.2f}, {start_yaw:.2f})")
			return Status.RUNNING

		# timeout handling
		if self.timeout_s is not None:
			elapsed = (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9
			if elapsed > self.timeout_s:
				if self._goal_handle is not None:
					try:
						self._goal_handle.cancel_goal_async()
					except Exception as exc:
						self.node.get_logger().warn(f"Cancel goal failed: {exc}")
				return Status.FAILURE

		if self._result_future is not None and self._result_future.done():
			result_stub = self._result_future.result()
			status_code = getattr(result_stub, 'status', None)
			if status_code is None and self._goal_handle is not None:
				status_code = getattr(self._goal_handle, 'status', None)
			if status_code == 4:  # STATUS_SUCCEEDED
				return Status.SUCCESS
			else:
				return Status.FAILURE

		return Status.RUNNING

	def terminate(self, new_status: Status) -> None:
		if new_status == Status.INVALID and self.cancel_on_terminate and self._goal_handle is not None:
			try:
				self._goal_handle.cancel_goal_async()
			except Exception as exc:
				self.node.get_logger().warn(f"Cancel goal failed on terminate: {exc}")

	def _on_goal_response(self, future):
		self._goal_handle = future.result()
		if not getattr(self._goal_handle, 'accepted', False):
			self._result_future = None
			self.node.get_logger().warn(f"NavigateToStartPosition goal rejected")
		else:
			self._result_future = self._goal_handle.get_result_async() 