#!/usr/bin/env python3
"""
Behavior Tree GUI debugger for YAML trees (e.g., nav.yaml, RMUL.yaml).

Features:
- Load YAML tree; visualize nodes and edges on a canvas (no Graphviz required).
- Step-by-step execution (generator) and per-tick evaluation.
- Toggle blackboard keys, params, and topics used by conditions.
- Highlights current node during stepping; colors by status.

Run:
  python3 tools/bt_gui.py src/rm_bt_decision/config/trees/nav.yaml

Dependencies: only standard library (tkinter, yaml via PyYAML is required to parse YAML).
Install PyYAML if missing: pip install pyyaml
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple, Iterable, Generator, Union

try:
    import yaml  # type: ignore
except Exception:
    print("PyYAML is required. Install with: pip install pyyaml", file=sys.stderr)
    raise

import tkinter as tk
from tkinter import ttk, filedialog, messagebox


# --------------------------
# Model
# --------------------------


_NodeStatus = Union[str, None]  # None|"IDLE"|"RUNNING"|"SUCCESS"|"FAILURE"


@dataclass
class BTNode:
    nid: int
    type: str
    name: Optional[str] = None
    params: Dict[str, Any] = field(default_factory=dict)
    children: List["BTNode"] = field(default_factory=list)
    decorators: List["BTNode"] = field(default_factory=list)

    # Runtime state
    status: _NodeStatus = None
    memory_index: int = 0  # for Memory Sequence/Selector
    one_shot_locked: bool = False  # for OneShot decorator policy
    wait_steps_left: Optional[int] = None  # for Wait simulation

    # Layout
    pos: Tuple[int, int] = (0, 0)  # grid row/col
    xy: Tuple[int, int] = (0, 0)  # pixel center


def load_tree(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or "tree" not in data:
        raise ValueError("YAML missing top-level 'tree' key")
    tree = data["tree"]
    if not isinstance(tree, dict) or "root" not in tree:
        raise ValueError("'tree' missing 'root' definition")
    return tree


def build_nodes(obj: Dict[str, Any], next_id: List[int]) -> BTNode:
    def new_id() -> int:
        nid = next_id[0]
        next_id[0] += 1
        return nid

    node = BTNode(
        nid=new_id(),
        type=str(obj.get("type", "Node")),
        name=obj.get("name"),
        params=dict(obj.get("params") or {}),
    )
    for c in obj.get("children") or []:
        node.children.append(build_nodes(c, next_id))
    for d in obj.get("decorators") or []:
        # Decorators as nodes that wrap this node during layout & evaluation
        node.decorators.append(
            BTNode(
                nid=new_id(),
                type=str(d.get("type", "Decorator")),
                name=d.get("name"),
                params=dict(d.get("params") or {}),
            )
        )
    return node


# --------------------------
# Evaluation (step/tick)
# --------------------------


class World:
    """Holds simulated runtime state (blackboard, params, topics)."""

    def __init__(self) -> None:
        self.bb: Dict[str, Any] = {}  # blackboard
        self.param: Dict[str, Any] = {}  # global params
        self.topic: Dict[str, Any] = {}  # topic last values

    def reset(self) -> None:
        # keep keys, reset values to defaults if bool -> False, else None
        for d in (self.bb, self.param, self.topic):
            for k in list(d.keys()):
                v = d[k]
                if isinstance(v, bool):
                    d[k] = False
                else:
                    d[k] = None


def init_world_from_yaml(root: BTNode, world: World) -> None:
    # scan nodes to pre-populate world keys
    def scan(n: BTNode) -> None:
        t = n.type
        p = n.params
        if t == "CheckBlackboard" and isinstance(p.get("key"), str):
            world.bb.setdefault(p["key"], False if isinstance(p.get("expected"), bool) else None)
        if t == "SetBlackboard" and isinstance(p.get("key"), str):
            world.bb.setdefault(p["key"], p.get("value"))
        if t in ("ParamBoolCondition", "WaitForParamBool") and isinstance(p.get("param"), str):
            world.param.setdefault(p["param"], False if isinstance(p.get("expected"), bool) else None)
        if t == "TopicStringEquals" and isinstance(p.get("topic"), str):
            world.topic.setdefault(p["topic"], "")
        for d in n.decorators:
            scan(d)
        for c in n.children:
            scan(c)

    scan(root)


def reset_runtime(node: BTNode) -> None:
    node.status = None
    node.memory_index = 0
    node.one_shot_locked = False
    node.wait_steps_left = None
    for d in node.decorators:
        reset_runtime(d)
    for c in node.children:
        reset_runtime(c)


def _bool(x: Any) -> Optional[bool]:
    if isinstance(x, bool):
        return x
    if x is None:
        return None
    if isinstance(x, (int, float)):
        return bool(x)
    if isinstance(x, str):
        lx = x.lower().strip()
        if lx in ("true", "1", "yes", "on"):  # tolerate strings
            return True
        if lx in ("false", "0", "no", "off", ""):
            return False
    return None


def tick_generator(root: BTNode, world: World) -> Generator[Tuple[str, BTNode], None, str]:
    """Yield (phase, node) where phase in {enter, exit} for each visit. Returns root status."""

    def visit(n: BTNode) -> str:
        # Decorators wrap the node. Build chain: deco_k(...deco_1(n))
        def apply_decorators(target: BTNode) -> str:
            # Apply decorators in order: first in list is outermost wrapper
            def run_decorator(idx: int, child: BTNode) -> str:
                if idx >= len(n.decorators):
                    return eval_node(child)
                deco = n.decorators[idx]
                yield_event("enter", deco)
                status: str
                if deco.type == "OneShot":
                    # Policy: ON_SUCCESSFUL_COMPLETION => once child succeeds, lock SUCCESS forever
                    if child.one_shot_locked:
                        status = "SUCCESS"
                    else:
                        status = run_decorator(idx + 1, child)
                        if status == "SUCCESS":
                            child.one_shot_locked = True
                else:
                    # Unknown decorators: transparent
                    status = run_decorator(idx + 1, child)
                yield_event("exit", deco)
                deco.status = status
                return status

            return run_decorator(0, target)

        def yield_event(phase: str, node: BTNode) -> None:
            events.append((phase, node))

        def eval_node(node: BTNode) -> str:
            yield_event("enter", node)
            t = node.type
            # Composite nodes
            if t in ("Sequence", "Selector"):
                memory = bool(node.params.get("memory", False))
                start = node.memory_index if memory else 0
                status = "SUCCESS" if t == "Sequence" else "FAILURE"
                for idx in range(start, len(node.children)):
                    child = node.children[idx]
                    child_status = visit(child)
                    if t == "Sequence":
                        if child_status == "RUNNING":
                            node.memory_index = idx if memory else 0
                            status = "RUNNING"
                            break
                        if child_status == "FAILURE":
                            status = "FAILURE"
                            break
                        # SUCCESS -> continue
                    else:  # Selector
                        if child_status == "RUNNING":
                            node.memory_index = idx if memory else 0
                            status = "RUNNING"
                            break
                        if child_status == "SUCCESS":
                            status = "SUCCESS"
                            break
                        # FAILURE -> continue
                if status != "RUNNING":
                    node.memory_index = 0  # reset when done
                node.status = status
                yield_event("exit", node)
                return status

            if t == "Parallel":
                policy = node.params.get("policy", "SuccessOnAll")
                succ = fail = run = 0
                for child in node.children:
                    s = visit(child)
                    if s == "SUCCESS":
                        succ += 1
                    elif s == "FAILURE":
                        fail += 1
                    else:
                        run += 1
                status: str
                if policy == "SuccessOnOne":
                    status = "SUCCESS" if succ >= 1 else ("FAILURE" if fail == len(node.children) else "RUNNING")
                else:  # SuccessOnAll
                    status = "SUCCESS" if succ == len(node.children) else ("FAILURE" if fail >= 1 else "RUNNING")
                node.status = status
                yield_event("exit", node)
                return status

            # Leaf nodes (simulate)
            status = eval_leaf(node, world)
            node.status = status
            yield_event("exit", node)
            return status

        return apply_decorators(n)

    def eval_leaf(node: BTNode, world: World) -> str:
        t = node.type
        p = node.params
        if t == "CheckBlackboard":
            key = p.get("key"); expected = p.get("expected")
            return "SUCCESS" if world.bb.get(key) == expected else "FAILURE"
        if t == "SetBlackboard":
            key = p.get("key"); value = p.get("value")
            if key is not None:
                world.bb[key] = value
            return "SUCCESS"
        if t == "ParamBoolCondition":
            prm = p.get("param"); expected = _bool(p.get("expected"))
            val = _bool(world.param.get(prm))
            return "SUCCESS" if (expected is not None and val == expected) else "FAILURE"
        if t == "WaitForParamBool":
            prm = p.get("param"); expected = _bool(p.get("expected"))
            val = _bool(world.param.get(prm))
            return "SUCCESS" if (expected is not None and val == expected) else "RUNNING"
        if t == "TopicStringEquals":
            topic = p.get("topic"); expected = p.get("expected")
            return "SUCCESS" if world.topic.get(topic) == expected else "FAILURE"
        if t == "Wait":
            # simulate wait as discrete steps ~= ceil(duration_s)
            dur = p.get("duration_s", 1.0)
            steps = max(1, int(math.ceil(float(dur))))
            if node.wait_steps_left is None:
                node.wait_steps_left = steps
            node.wait_steps_left -= 1
            return "SUCCESS" if node.wait_steps_left <= 0 else "RUNNING"
        # Default for action-like nodes: SUCCESS immediately
        return "SUCCESS"

    # flatten events via list accumulation so that generator can yield sequentially
    events: List[Tuple[str, BTNode]] = []
    result = visit(root)
    for e in events:
        yield e
    return result


# --------------------------
# Layout
# --------------------------


def compute_layout(root: BTNode) -> List[BTNode]:
    """Assign grid positions and compute pixel positions for a left-to-right tree layout.

    Decorators are placed to the left of their target node in a chain.
    """
    levels: Dict[int, List[BTNode]] = {}

    def place(node: BTNode, depth: int, row: List[int]) -> None:
        # Place decorators chain first, increasing depth to the left
        dprev: Optional[BTNode] = None
        for deco in node.decorators:
            deco.pos = (row[0], depth)
            levels.setdefault(depth, []).append(deco)
            row[0] += 1
            if dprev is not None:
                pass
            dprev = deco
            depth += 1
        # Place main node
        node.pos = (row[0], depth)
        levels.setdefault(depth, []).append(node)
        row[0] += 1
        # Recurse children, deeper depth
        child_depth = depth + 1
        for ch in node.children:
            place(ch, child_depth, row)

    place(root, 0, [0])

    # compute pixel coordinates
    x_gap, y_gap = 200, 90
    for depth, nodes in levels.items():
        for i, n in enumerate(nodes):
            x = 80 + depth * x_gap
            y = 60 + i * y_gap
            n.xy = (x, y)

    # Return draw order (by depth then row)
    ordered: List[BTNode] = []
    for d in sorted(levels.keys()):
        ordered.extend(levels[d])
    return ordered


# --------------------------
# GUI
# --------------------------


class BTDebuggerApp:
    def __init__(self, master: tk.Tk, path: Optional[str] = None) -> None:
        self.master = master
        master.title("BT GUI Debugger")
        master.geometry("1200x800")

        self.world = World()
        self.tree_name: str = ""
        self.root_node: Optional[BTNode] = None
        self.step_gen: Optional[Generator[Tuple[str, BTNode], None, str]] = None
        self.last_result: Optional[str] = None

        self._build_ui()
        if path:
            self.load_yaml(path)

    # UI setup
    def _build_ui(self) -> None:
        self.frm = ttk.Frame(self.master)
        self.frm.pack(fill=tk.BOTH, expand=True)

        # Left: canvas
        self.canvas = tk.Canvas(self.frm, bg="#ffffff")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Right: controls
        right = ttk.Frame(self.frm, width=360)
        right.pack(side=tk.RIGHT, fill=tk.Y)

        # File controls
        file_bar = ttk.Frame(right)
        file_bar.pack(fill=tk.X, pady=6)
        ttk.Button(file_bar, text="Open YAML", command=self.open_yaml).pack(side=tk.LEFT)
        self.file_label = ttk.Label(file_bar, text="(no file)")
        self.file_label.pack(side=tk.LEFT, padx=8)

        # Execution controls
        exec_bar = ttk.Frame(right)
        exec_bar.pack(fill=tk.X, pady=6)
        ttk.Button(exec_bar, text="Step", command=self.step_once).pack(side=tk.LEFT)
        ttk.Button(exec_bar, text="Tick", command=self.tick_once).pack(side=tk.LEFT, padx=6)
        ttk.Button(exec_bar, text="Reset", command=self.reset_runtime).pack(side=tk.LEFT)
        self.result_label = ttk.Label(exec_bar, text="result: -")
        self.result_label.pack(side=tk.LEFT, padx=8)

        # World state controls
        self.nb = ttk.Notebook(right)
        self.nb.pack(fill=tk.BOTH, expand=True)
        self.tab_bb = ttk.Frame(self.nb)
        self.tab_param = ttk.Frame(self.nb)
        self.tab_topic = ttk.Frame(self.nb)
        self.nb.add(self.tab_bb, text="Blackboard")
        self.nb.add(self.tab_param, text="Params")
        self.nb.add(self.tab_topic, text="Topics")

        self.bb_vars: Dict[str, tk.Variable] = {}
        self.param_vars: Dict[str, tk.Variable] = {}
        self.topic_vars: Dict[str, tk.Variable] = {}

        # Status legend
        legend = ttk.Frame(right)
        legend.pack(fill=tk.X, pady=8)
        for col in [("IDLE", "#d0d0d0"), ("RUNNING", "#ffd860"), ("SUCCESS", "#a6e3a1"), ("FAILURE", "#f38ba8")]:
            c = tk.Canvas(legend, width=16, height=16)
            c.create_rectangle(1, 1, 15, 15, fill=col[1], outline="#888")
            c.pack(side=tk.LEFT, padx=4)
            ttk.Label(legend, text=col[0]).pack(side=tk.LEFT)

    # File operations
    def open_yaml(self) -> None:
        path = filedialog.askopenfilename(title="Open BT YAML", filetypes=[("YAML", "*.yaml *.yml"), ("All", "*.*")])
        if not path:
            return
        self.load_yaml(path)

    def load_yaml(self, path: str) -> None:
        try:
            tree = load_tree(path)
            self.tree_name = str(tree.get("name", "BehaviorTree"))
            self.root_node = build_nodes(tree["root"], [0])
            init_world_from_yaml(self.root_node, self.world)
            self.populate_world_tabs()
            self.layout_and_draw()
            self.file_label.configure(text=os.path.basename(path))
            self.step_gen = None
            self.last_result = None
            self.result_label.configure(text="result: -")
        except Exception as e:
            messagebox.showerror("Load Error", str(e))

    # World tab population
    def _populate_tab(self, tab: ttk.Frame, mapping: Dict[str, Any], store: Dict[str, tk.Variable]) -> None:
        for child in list(tab.children.values()):
            child.destroy()
        # sort keys for stable UI
        keys = sorted(mapping.keys())
        for i, k in enumerate(keys):
            v = mapping[k]
            ttk.Label(tab, text=k).grid(row=i, column=0, sticky=tk.W, padx=4, pady=2)
            if isinstance(v, bool) or v is None:
                var = tk.BooleanVar(value=bool(v))
                store[k] = var
                cb = ttk.Checkbutton(tab, variable=var)
                cb.grid(row=i, column=1, sticky=tk.W)
            else:
                var = tk.StringVar(value=str(v))
                store[k] = var
                ent = ttk.Entry(tab, textvariable=var, width=18)
                ent.grid(row=i, column=1, sticky=tk.W)

        # Add apply button
        def apply_values() -> None:
            for k, var in store.items():
                if isinstance(var, tk.BooleanVar):
                    mapping[k] = bool(var.get())
                else:
                    mapping[k] = var.get()
        ttk.Button(tab, text="Apply", command=apply_values).grid(row=len(keys)+1, column=0, pady=6)

    def populate_world_tabs(self) -> None:
        self._populate_tab(self.tab_bb, self.world.bb, self.bb_vars)
        self._populate_tab(self.tab_param, self.world.param, self.param_vars)
        self._populate_tab(self.tab_topic, self.world.topic, self.topic_vars)

    # Execution controls
    def reset_runtime(self) -> None:
        if self.root_node is None:
            return
        reset_runtime(self.root_node)
        self.world.reset()
        self.populate_world_tabs()
        self.step_gen = None
        self.last_result = None
        self.result_label.configure(text="result: -")
        self.layout_and_draw()

    def step_once(self) -> None:
        if self.root_node is None:
            return
        if self.step_gen is None:
            self.step_gen = tick_generator(self.root_node, self.world)
        try:
            phase, node = next(self.step_gen)
            # draw with current highlight
            self.layout_and_draw(current=node)
        except StopIteration as it:
            result = it.value if hasattr(it, "value") else None
            self.last_result = result
            self.result_label.configure(text=f"result: {result}")
            self.step_gen = None
            self.layout_and_draw()

    def tick_once(self) -> None:
        if self.root_node is None:
            return
        gen = tick_generator(self.root_node, self.world)
        last_node: Optional[BTNode] = None
        try:
            while True:
                phase, node = next(gen)
                last_node = node
        except StopIteration as it:
            result = it.value
            self.last_result = result
        self.result_label.configure(text=f"result: {self.last_result}")
        self.layout_and_draw(current=last_node)

    # Drawing
    def layout_and_draw(self, current: Optional[BTNode] = None) -> None:
        if self.root_node is None:
            return
        self.canvas.delete("all")
        nodes = compute_layout(self.root_node)

        # Draw edges: from parent chain: decorators -> node -> children
        def draw_edge(a: BTNode, b: BTNode) -> None:
            ax, ay = a.xy; bx, by = b.xy
            self.canvas.create_line(ax+60, ay, bx-60, by, arrow=tk.LAST, fill="#555")

        # For each node, connect its decorators sequentially and to the node
        def connect_decorators(n: BTNode) -> None:
            prev = None
            for deco in n.decorators:
                if prev is not None:
                    draw_edge(prev, deco)
                prev = deco
            if prev is not None:
                draw_edge(prev, n)
        def walk(n: BTNode) -> None:
            connect_decorators(n)
            for c in n.children:
                walk(c)
        walk(self.root_node)

        # For the whole tree, also connect parent to first decorator or node
        def traverse(parent: Optional[BTNode], n: BTNode) -> None:
            attach = n.decorators[0] if n.decorators else n
            if parent is not None:
                draw_edge(parent, attach)
            # connect inner decorator chain already handled
            for c in n.children:
                traverse(n, c)

        traverse(None, self.root_node)

        # Draw nodes
        for n in nodes:
            self.draw_node(n, highlight=(current is not None and n.nid == current.nid))

    def draw_node(self, n: BTNode, highlight: bool = False) -> None:
        x, y = n.xy
        w, h = (120, 48)
        status = n.status or "IDLE"
        fill = {
            "IDLE": "#eaeaea",
            "RUNNING": "#ffd860",
            "SUCCESS": "#a6e3a1",
            "FAILURE": "#f38ba8",
        }.get(status, "#eaeaea")

        outline = "#1e66f5" if highlight else "#444"
        shape = "rect"
        if n.type in ("Selector", "Sequence", "Parallel") or n.children:
            shape = "roundrect"
        if n.type in ("OneShot",):
            shape = "diamond"

        if shape == "diamond":
            pts = [x, y - h/2, x + w/2, y, x, y + h/2, x - w/2, y]
            self.canvas.create_polygon(pts, fill=fill, outline=outline, width=2)
        else:
            r = 10 if shape == "roundrect" else 2
            self._rounded_rect(x - w/2, y - h/2, x + w/2, y + h/2, r, fill=fill, outline=outline, width=2)

        title = n.type if not n.name or n.name == n.type else f"{n.type}\n{n.name}"
        self.canvas.create_text(x, y, text=title, font=("Helvetica", 10))

    def _rounded_rect(self, x1: float, y1: float, x2: float, y2: float, r: float, **kwargs: Any) -> None:
        # Draw a rounded rectangle on the canvas
        self.canvas.create_arc(x1, y1, x1 + 2*r, y1 + 2*r, start=90, extent=90, style=tk.PIESLICE, **kwargs)
        self.canvas.create_arc(x2 - 2*r, y1, x2, y1 + 2*r, start=0, extent=90, style=tk.PIESLICE, **kwargs)
        self.canvas.create_arc(x2 - 2*r, y2 - 2*r, x2, y2, start=270, extent=90, style=tk.PIESLICE, **kwargs)
        self.canvas.create_arc(x1, y2 - 2*r, x1 + 2*r, y2, start=180, extent=90, style=tk.PIESLICE, **kwargs)
        self.canvas.create_rectangle(x1 + r, y1, x2 - r, y2, **kwargs)
        self.canvas.create_rectangle(x1, y1 + r, x2, y2 - r, **kwargs)


def main(argv: Optional[List[str]] = None) -> int:
    path = None
    if argv is None:
        argv = sys.argv[1:]
    if argv:
        path = argv[0]
        if not os.path.isfile(path):
            print(f"File not found: {path}", file=sys.stderr)
            return 2

    root = tk.Tk()
    app = BTDebuggerApp(root, path)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
