from __future__ import annotations

import os
from typing import Optional

import rclpy
from rclpy.node import Node
import py_trees

try:
	import py_trees_ros.trees as ros_trees
	_HAS_PY_TREES_ROS = True
except Exception:
	_HAS_PY_TREES_ROS = False

from .loader import build_root_from_yaml


class BTNode(Node):
	def __init__(self):
		super().__init__('rm_bt_decision_node')
		self.declare_parameter('tree_config', '')
		self.declare_parameter('tick_hz', 10.0)
		self.declare_parameter('use_web_viewer', True)
		self.declare_parameter('enable_text_output', True)
		self.declare_parameter('text_output_every_n', 1)
		tree_config: str = self.get_parameter('tree_config').get_parameter_value().string_value
		self.tick_hz: float = self.get_parameter('tick_hz').get_parameter_value().double_value
		self.use_web_viewer: bool = self.get_parameter('use_web_viewer').get_parameter_value().bool_value
		self.enable_text_output: bool = self.get_parameter('enable_text_output').get_parameter_value().bool_value
		self.text_output_every_n: int = int(self.get_parameter('text_output_every_n').get_parameter_value().integer_value)

		if not tree_config:
			# fallback to installed share path
			share = os.path.join(os.path.dirname(__file__), '..', 'config', 'trees', 'nav.yaml')
			self.tree_config = os.path.abspath(share)
		else:
			self.tree_config = tree_config

		self.get_logger().info(f"Loading BT from: {self.tree_config}")
		root = build_root_from_yaml(self, self.tree_config)

		# Attach snapshot visitor for ascii/unicode text rendering
		self.snapshot_visitor = py_trees.visitors.SnapshotVisitor()
		self._tick_counter = 0

		if _HAS_PY_TREES_ROS and self.use_web_viewer:
			self.get_logger().info("Using py_trees_ros BehaviourTree for web visualization")
			self.tree = ros_trees.BehaviourTree(root)
			self.tree.visitors.append(self.snapshot_visitor)
			self._attach_text_output_handlers()
			self.tree.setup(timeout_sec=3.0)
			# Use ROS timer to tick instead of blocking tick_tock
			self.timer = self.create_timer(1.0 / max(1e-3, self.tick_hz), self._on_timer)
		else:
			self.get_logger().warn("py_trees_ros not available or web viewer disabled; using plain py_trees")
			self.tree = py_trees.trees.BehaviourTree(root)
			self.tree.visitors.append(self.snapshot_visitor)
			self._attach_text_output_handlers()
			self.timer = self.create_timer(1.0 / max(1e-3, self.tick_hz), self._on_timer)

	def _attach_text_output_handlers(self):
		if not self.enable_text_output:
			return
		def _post_tick(tree: py_trees.trees.BehaviourTree):
			self._tick_counter += 1
			if self.text_output_every_n <= 0:
				return
			if (self._tick_counter % self.text_output_every_n) != 0:
				return
			# Use a widely supported display api
			try:
				text = py_trees.display.unicode_tree(root=tree.root, show_status=True)
			except TypeError:
				# Fallback if unicode_tree signature differs
				text = py_trees.display.unicode_tree(tree.root)
			self.get_logger().info("\n" + text)
		# Prefer official api if available
		if hasattr(self.tree, 'add_post_tick_handler'):
			self.tree.add_post_tick_handler(_post_tick)
		else:
			# Fallback to direct list append
			if hasattr(self.tree, 'post_tick_handlers'):
				self.tree.post_tick_handlers.append(_post_tick)

	def _on_timer(self):
		self.tree.tick()


def main(args=None):
	rclpy.init(args=args)
	node = BTNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown() 