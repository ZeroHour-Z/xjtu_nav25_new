from typing import Optional

import py_trees
from py_trees.common import Status
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String as StringMsg

from ..registry import register


@register("PublishString")
class PublishString(py_trees.behaviour.Behaviour):
	def __init__(
		self,
		name: str,
		node: Node,
		topic: str,
		data: str,
		qos_depth: int = 10,
		reliable: bool = True,
		transient_local: bool = False,
		publish_every_tick: bool = False,
	):
		super().__init__(name)
		self.node = node
		self.topic = topic
		self.data = data
		self.qos_depth = int(qos_depth)
		self.reliable = bool(reliable)
		self.transient_local = bool(transient_local)
		self.publish_every_tick = bool(publish_every_tick)
		self._pub = None
		self._published_once = False

	def setup(self, **kwargs) -> None:
		reliability = ReliabilityPolicy.RELIABLE if self.reliable else ReliabilityPolicy.BEST_EFFORT
		durability = DurabilityPolicy.TRANSIENT_LOCAL if self.transient_local else DurabilityPolicy.VOLATILE
		qos = QoSProfile(depth=self.qos_depth, reliability=reliability, durability=durability)
		self._pub = self.node.create_publisher(StringMsg, self.topic, qos)

	def initialise(self) -> None:
		self._published_once = False

	def update(self) -> Status:
		if self._pub is None:
			return Status.FAILURE
		msg = StringMsg()
		msg.data = str(self.data)
		if self.publish_every_tick:
			self._pub.publish(msg)
			return Status.SUCCESS
		else:
			if not self._published_once:
				self._pub.publish(msg)
				self._published_once = True
			return Status.SUCCESS 