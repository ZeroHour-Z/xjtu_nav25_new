from typing import Callable, Dict, Type

import py_trees


class BehaviourRegistry:
	_registry: Dict[str, Type[py_trees.behaviour.Behaviour]] = {}

	@classmethod
	def register(cls, name: str, behaviour_cls: Type[py_trees.behaviour.Behaviour]) -> None:
		if name in cls._registry:
			raise KeyError(f"Behaviour '{name}' already registered")
		cls._registry[name] = behaviour_cls

	@classmethod
	def get(cls, name: str) -> Type[py_trees.behaviour.Behaviour]:
		if name not in cls._registry:
			raise KeyError(f"Behaviour '{name}' not found. Registered: {list(cls._registry.keys())}")
		return cls._registry[name]


def register(name: str):
	def _decorator(behaviour_cls: Type[py_trees.behaviour.Behaviour]):
		BehaviourRegistry.register(name, behaviour_cls)
		return behaviour_cls
	return _decorator 