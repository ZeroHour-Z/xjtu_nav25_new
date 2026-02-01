from __future__ import annotations

from typing import Any, Dict, List, Optional

import importlib
import argparse
import py_trees
import yaml

# Trigger registration of built-in behaviours/actions
from . import behaviors as _bt_behaviors  # noqa: F401
from . import actions as _bt_actions  # noqa: F401

from .registry import BehaviourRegistry


class RetryDecorator(py_trees.decorators.Decorator):
	def __init__(self, name: str, child: py_trees.behaviour.Behaviour, num_retries: int = 1):
		super().__init__(name=name, child=child)
		self.num_retries = max(0, int(num_retries))
		self._attempts = 0

	def initialise(self) -> None:
		self._attempts = 0

	def update(self) -> py_trees.common.Status:
		child_status = self.decorated.status
		if child_status == py_trees.common.Status.INVALID:
			return py_trees.common.Status.RUNNING
		if child_status == py_trees.common.Status.FAILURE and self._attempts < self.num_retries:
			self._attempts += 1
			self.decorated.stop(py_trees.common.Status.INVALID)
			self.decorated.tick_once()
			return py_trees.common.Status.RUNNING
		return child_status


class TimeoutDecorator(py_trees.decorators.Decorator):
	def __init__(self, name: str, child: py_trees.behaviour.Behaviour, duration_s: float):
		super().__init__(name=name, child=child)
		self.duration_s = float(duration_s)
		self._start = None
		self._node = None

	def setup(self, **kwargs) -> None:
		# Try to find a ROS node in child attribute space for timing
		self._node = getattr(self.decorated, 'node', None)

	def initialise(self) -> None:
		if self._node is not None:
			self._start = self._node.get_clock().now()
		else:
			self._start = None

	def update(self) -> py_trees.common.Status:
		if self._start is None:
			return self.decorated.status
		elapsed = (self._node.get_clock().now() - self._start).nanoseconds / 1e9
		if elapsed > self.duration_s:
			self.decorated.stop(py_trees.common.Status.INVALID)
			return py_trees.common.Status.FAILURE
		return self.decorated.status


def build_behaviour_from_dict(spec: Dict[str, Any], ros_node) -> py_trees.behaviour.Behaviour:
	node_type = spec["type"]
	name = spec.get("name", node_type)
	params = spec.get("params", {})
	decorators = spec.get("decorators", [])

	# Composites
	if node_type in ("Sequence", "Selector", "Parallel"):
		children_specs = spec.get("children", [])
		children = [build_behaviour_from_dict(c, ros_node) for c in children_specs]
		if node_type == "Sequence":
			memory = bool(params.get("memory", False))
			composite = py_trees.composites.Sequence(name=name, memory=memory)
			composite.add_children(children)
		elif node_type == "Selector":
			memory = bool(params.get("memory", False))
			composite = py_trees.composites.Selector(name=name, memory=memory)
			composite.add_children(children)
		else:
			policy_name = params.get("policy", "SuccessOnAll")
			synchronise = bool(params.get("synchronise", False))
			# Build policy with compatibility across py_trees versions
			if policy_name == "SuccessOnOne":
				try:
					policy = py_trees.common.ParallelPolicy.SuccessOnOne(synchronise=synchronise)
				except TypeError:
					policy = py_trees.common.ParallelPolicy.SuccessOnOne()
					try:
						policy.synchronise = synchronise
					except Exception:
						pass
			else:
				try:
					policy = py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=synchronise)
				except TypeError:
					policy = py_trees.common.ParallelPolicy.SuccessOnAll()
					try:
						policy.synchronise = synchronise
					except Exception:
						pass
			composite = py_trees.composites.Parallel(name=name, policy=policy)
			composite.add_children(children)
		return apply_decorators(composite, decorators)

	# Leaves (behaviours)
	behaviour_cls = BehaviourRegistry.get(node_type)
	instance = behaviour_cls(name=name, node=ros_node, **params)
	return apply_decorators(instance, decorators)


def apply_decorators(behaviour: py_trees.behaviour.Behaviour, decorators: List[Dict[str, Any]]):
	wrapped = behaviour
	for idx, deco in enumerate(decorators or []):
		type_name = deco.get("type")
		params = deco.get("params", {})
		name = deco.get("name", f"{type_name}_{idx}")
		if type_name == "Retry":
			wrapped = RetryDecorator(name=name, child=wrapped, **params)
		elif type_name == "Timeout":
			wrapped = TimeoutDecorator(name=name, child=wrapped, **params)
		elif type_name == "Inverter":
			wrapped = py_trees.decorators.Inverter(name=name, child=wrapped)
		elif type_name == "OneShot":
			policy_str = params.get("policy", "ON_SUCCESSFUL_COMPLETION")
			try:
				policy = getattr(py_trees.common.OneShotPolicy, policy_str)
			except Exception:
				policy = py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
			wrapped = py_trees.decorators.OneShot(name=name, child=wrapped, policy=policy)
		else:
			raise ValueError(f"Unknown decorator: {type_name}")
	return wrapped


def build_root_from_yaml(ros_node, yaml_path: str) -> py_trees.behaviour.Behaviour:
	with open(yaml_path, 'r') as f:
		data = yaml.safe_load(f)
	tree_def = data.get("tree") or data
	return build_behaviour_from_dict(tree_def.get("root", tree_def), ros_node)


def render_main():
	parser = argparse.ArgumentParser()
	parser.add_argument('--tree', required=True, help='YAML tree path')
	args = parser.parse_args()
	# Minimal render without ROS node
	class Dummy:
		def get_clock(self):
			class C:
				def now(self):
					import rclpy
					return rclpy.time.Time()
			return C()
	root = build_root_from_yaml(Dummy(), args.tree)
	dot = py_trees.display.render_dot_tree(root)
	print(dot) 