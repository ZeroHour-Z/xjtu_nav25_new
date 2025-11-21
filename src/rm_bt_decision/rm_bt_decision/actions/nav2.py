from typing import Optional
import math

import py_trees
from py_trees.common import Status
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy import time as rclpy_time
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

from ..registry import register


@register("NavigateToPoseAction")
class NavigateToPoseAction(py_trees.behaviour.Behaviour):
	def __init__(
		self,
		name: str,
		node: Node,
		server_name: str = "navigate_to_pose",
		goal_pose: Optional[PoseStamped] = None,
		frame_id: str = "map",
		x: float = 0.0,
		y: float = 0.0,
		yaw: float = 0.0,
		x_param: Optional[str] = None,  # 如果提供，从参数读取x
		y_param: Optional[str] = None,  # 如果提供，从参数读取y
		yaw_param: Optional[str] = None,  # 如果提供，从参数读取yaw
		timeout_s: Optional[float] = None,
		cancel_on_terminate: bool = True,
	):
		super().__init__(name)
		self.node = node
		self.client = ActionClient(node, NavigateToPose, server_name)
		self._goal_handle = None
		self._result_future = None
		self._sent = False
		self.timeout_s = timeout_s
		self.cancel_on_terminate = cancel_on_terminate
		self._start_time = None
		self._goal_pose = goal_pose
		self._frame_id = frame_id
		self._x = x
		self._y = y
		self._yaw = yaw
		self._x_param = x_param
		self._y_param = y_param
		self._yaw_param = yaw_param

	def setup(self, **kwargs) -> None:
		# kwargs may include timeout, node, visitor. Use timeout if provided; otherwise block briefly.
		timeout_sec = float(kwargs.get('timeout', 3.0)) if 'timeout' in kwargs else 3.0
		if not self.client.wait_for_server(timeout_sec=timeout_sec):
			raise RuntimeError("Nav2 NavigateToPose action server not available")

	def initialise(self) -> None:
		self._sent = False
		self._goal_handle = None
		self._result_future = None
		self._start_time = self.node.get_clock().now()

	def update(self) -> Status:
		if not self._sent:
			goal_msg = NavigateToPose.Goal()
			if self._goal_pose is None:
				# 如果提供了参数名，从参数读取；否则使用固定值
				x_val = self._x
				y_val = self._y
				yaw_val = self._yaw
				
				if self._x_param:
					try:
						x_val = float(self.node.get_parameter(self._x_param).value)
					except Exception:
						self.node.get_logger().warn(f"Failed to read x from param {self._x_param}, using default {self._x}")
				if self._y_param:
					try:
						y_val = float(self.node.get_parameter(self._y_param).value)
					except Exception:
						self.node.get_logger().warn(f"Failed to read y from param {self._y_param}, using default {self._y}")
				if self._yaw_param:
					try:
						yaw_val = float(self.node.get_parameter(self._yaw_param).value)
					except Exception:
						self.node.get_logger().warn(f"Failed to read yaw from param {self._yaw_param}, using default {self._yaw}")
				
				pose = PoseStamped()
				pose.header.frame_id = self._frame_id
				pose.header.stamp = self.node.get_clock().now().to_msg()
				pose.pose.position.x = float(x_val)
				pose.pose.position.y = float(y_val)
				# simple yaw to quaternion (z,w)
				import math
				pose.pose.orientation.z = math.sin(yaw_val * 0.5)
				pose.pose.orientation.w = math.cos(yaw_val * 0.5)
				goal_msg.pose = pose
			else:
				goal_msg.pose = self._goal_pose
			send_future = self.client.send_goal_async(goal_msg)
			send_future.add_done_callback(self._on_goal_response)
			self._sent = True
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
			# Prefer result status when available
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
			self.node.get_logger().warn(f"NavigateToPose goal rejected")
		else:
			self._result_future = self._goal_handle.get_result_async() 


@register("NavigateToRelativePoseAction")
class NavigateToRelativePoseAction(py_trees.behaviour.Behaviour):
	"""导航到相对于当前位置的目标点（适用于未知区域导航）"""
	def __init__(
		self,
		name: str,
		node: Node,
		server_name: str = "navigate_to_pose",
		relative_x: float = 0.0,
		relative_y: float = 0.0,
		relative_yaw: float = 0.0,
		base_frame: str = "base_link",
		map_frame: str = "map",
		timeout_s: Optional[float] = None,
		cancel_on_terminate: bool = True,
	):
		super().__init__(name)
		self.node = node
		self.client = ActionClient(node, NavigateToPose, server_name)
		self._goal_handle = None
		self._result_future = None
		self._sent = False
		self.timeout_s = timeout_s
		self.cancel_on_terminate = cancel_on_terminate
		self._start_time = None
		self.relative_x = relative_x
		self.relative_y = relative_y
		self.relative_yaw = relative_yaw
		self.base_frame = base_frame
		self.map_frame = map_frame
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, node)

	def setup(self, **kwargs) -> None:
		timeout_sec = float(kwargs.get('timeout', 3.0)) if 'timeout' in kwargs else 3.0
		if not self.client.wait_for_server(timeout_sec=timeout_sec):
			raise RuntimeError("Nav2 NavigateToPose action server not available")

	def initialise(self) -> None:
		self._sent = False
		self._goal_handle = None
		self._result_future = None
		self._start_time = self.node.get_clock().now()

	def update(self) -> Status:
		if not self._sent:
			# 获取当前位姿
			try:
				transform = self.tf_buffer.lookup_transform(
					self.map_frame, self.base_frame, rclpy_time.Time()
				)
			except Exception as e:
				self.node.get_logger().warn(f"TF lookup failed: {e}")
				return Status.RUNNING

			# 计算当前位姿
			current_x = transform.transform.translation.x
			current_y = transform.transform.translation.y
			q = transform.transform.rotation
			# 计算当前yaw角
			siny_cosp = 2 * (q.w * q.z + q.x * q.y)
			cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
			current_yaw = math.atan2(siny_cosp, cosy_cosp)

			# 计算目标位置（在map坐标系下）
			# 相对位置需要先转换到map坐标系
			cos_yaw = math.cos(current_yaw)
			sin_yaw = math.sin(current_yaw)
			target_x = current_x + cos_yaw * self.relative_x - sin_yaw * self.relative_y
			target_y = current_y + sin_yaw * self.relative_x + cos_yaw * self.relative_y
			target_yaw = current_yaw + self.relative_yaw

			# 创建目标位姿
			goal_msg = NavigateToPose.Goal()
			pose = PoseStamped()
			pose.header.frame_id = self.map_frame
			pose.header.stamp = self.node.get_clock().now().to_msg()
			pose.pose.position.x = target_x
			pose.pose.position.y = target_y
			pose.pose.position.z = 0.0
			pose.pose.orientation.z = math.sin(target_yaw * 0.5)
			pose.pose.orientation.w = math.cos(target_yaw * 0.5)
			goal_msg.pose = pose

			self.node.get_logger().info(
				f"Navigating to relative goal: ({self.relative_x:.2f}, {self.relative_y:.2f}, {self.relative_yaw:.2f}) "
				f"-> absolute: ({target_x:.2f}, {target_y:.2f}, {target_yaw:.2f})"
			)

			send_future = self.client.send_goal_async(goal_msg)
			send_future.add_done_callback(self._on_goal_response)
			self._sent = True
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
			self.node.get_logger().warn(f"NavigateToRelativePose goal rejected")
		else:
			self._result_future = self._goal_handle.get_result_async()