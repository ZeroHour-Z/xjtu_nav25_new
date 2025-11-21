#!/usr/bin/env python3
"""
Visualize behavior tree YAML (e.g., nav.yaml, RMUL.yaml) as a Graphviz graph.

Usage:
  python tools/bt_visualize.py path/to/tree.yaml -o out.dot
  python tools/bt_visualize.py path/to/tree.yaml -o out.png --format png

Notes:
  - Requires PyYAML (`pip install pyyaml`).
  - For PNG/SVG output, requires Graphviz `dot` installed in PATH.
"""

from __future__ import annotations

import argparse
import os
import sys
import textwrap
from typing import Any, Dict, List, Optional, Tuple

try:
    import yaml  # type: ignore
except Exception as e:  # pragma: no cover
    print("[ERROR] PyYAML not installed. Install via: pip install pyyaml", file=sys.stderr)
    raise

import subprocess


def _escape(s: str) -> str:
    """Escape strings for DOT labels."""
    return (
        s.replace("\\", "\\\\")
        .replace("\n", "\\n")
        .replace("\"", "\\\"")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


class DotBuilder:
    def __init__(self) -> None:
        self.lines: List[str] = []
        self.lines.append("digraph BehaviorTree {")
        # aesthetics
        self.lines.append("  rankdir=LR;")
        self.lines.append("  node [fontname=\"Helvetica\", fontsize=11];")
        self.lines.append("  edge [fontname=\"Helvetica\", fontsize=10];")
        self._next_id = 0

    def new_id(self) -> str:
        nid = f"n{self._next_id}"
        self._next_id += 1
        return nid

    def add_node(self, node_id: str, label: str, shape: str = "box", style: Optional[str] = None) -> None:
        label = _escape(label)
        style_part = f", style={style}" if style else ""
        self.lines.append(f"  {node_id} [label=\"{label}\", shape={shape}{style_part}];")

    def add_edge(self, src_id: str, dst_id: str, label: Optional[str] = None) -> None:
        if label:
            self.lines.append(f"  {src_id} -> {dst_id} [label=\"{_escape(label)}\"];")
        else:
            self.lines.append(f"  {src_id} -> {dst_id};")

    def finish(self) -> str:
        self.lines.append("}")
        return "\n".join(self.lines) + "\n"


def node_style(node_type: str, has_children: bool, is_decorator: bool) -> Tuple[str, Optional[str]]:
    """Return (shape, style) for a node based on its type and role."""
    if is_decorator:
        return ("diamond", None)
    if node_type in ("Selector", "Sequence", "Parallel") or has_children:
        # composites
        return ("box", "rounded")
    # leaves
    return ("ellipse", None)


def node_label(node_type: str, node_name: Optional[str], params: Optional[Dict[str, Any]]) -> str:
    title = node_name or node_type
    if node_name and node_name != node_type:
        header = f"{node_type}\\n{node_name}"
    else:
        header = node_type

    # add a few high-signal params when present
    highlights: List[str] = []
    if isinstance(params, dict):
        if node_type in ("Selector", "Sequence") and "memory" in params:
            highlights.append(f"memory={params.get('memory')}")
        if node_type == "Parallel":
            if "policy" in params:
                highlights.append(f"policy={params.get('policy')}")
            if "synchronise" in params:
                highlights.append(f"sync={params.get('synchronise')}")

    if highlights:
        return header + "\\n" + " ".join(highlights)
    return header


def build_graph(dot: DotBuilder, node: Dict[str, Any]) -> str:
    """Recursively add nodes/edges to DOT. Returns the visible node_id to attach to its parent.

    Handles decorators by creating a wrapper chain: parent -> first_decorator -> ... -> original_node
    """
    node_type = str(node.get("type", "Node"))
    node_name = node.get("name")
    params = node.get("params")
    children = node.get("children") or []
    decorators = node.get("decorators") or []

    # Create main node first
    main_id = dot.new_id()
    shape, style = node_style(node_type, bool(children), is_decorator=False)
    dot.add_node(main_id, node_label(node_type, node_name, params), shape=shape, style=style)

    # Recursively add children for composites
    for idx, child in enumerate(children):
        child_id = build_graph(dot, child)
        dot.add_edge(main_id, child_id, label=None)

    # Decorators wrap the node; build chain from first to last
    attach_id = main_id
    if decorators:
        # We attach parent to the first decorator instead of the main node
        prev_id = None
        for deco in decorators:
            deco_type = str(deco.get("type", "Decorator"))
            deco_name = deco.get("name")
            deco_params = deco.get("params")
            deco_id = dot.new_id()
            dshape, dstyle = node_style(deco_type, False, is_decorator=True)
            dot.add_node(deco_id, node_label(deco_type, deco_name, deco_params), shape=dshape, style=dstyle)
            if prev_id is None:
                # This first decorator points to the main node
                dot.add_edge(deco_id, main_id)
            else:
                dot.add_edge(prev_id, deco_id)
            prev_id = deco_id
        attach_id = prev_id or main_id

    return attach_id


def render_dot(dot_text: str, out_path: str, fmt: str) -> None:
    if fmt == "dot":
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(dot_text)
        return

    # Render image via Graphviz `dot`
    if not shutil_which("dot"):
        print(
            "[WARN] Graphviz 'dot' not found in PATH. Writing DOT text instead.",
            file=sys.stderr,
        )
        dot_fallback = os.path.splitext(out_path)[0] + ".dot"
        with open(dot_fallback, "w", encoding="utf-8") as f:
            f.write(dot_text)
        print(f"[INFO] Wrote: {dot_fallback}")
        return

    cmd = ["dot", f"-T{fmt}", "-o", out_path]
    try:
        proc = subprocess.run(cmd, input=dot_text.encode("utf-8"), check=True)
    except subprocess.CalledProcessError as e:  # pragma: no cover
        print(f"[ERROR] dot failed: {e}", file=sys.stderr)
        # Fallback to .dot file
        dot_fallback = os.path.splitext(out_path)[0] + ".dot"
        with open(dot_fallback, "w", encoding="utf-8") as f:
            f.write(dot_text)
        print(f"[INFO] Wrote: {dot_fallback}")


def shutil_which(cmd: str) -> Optional[str]:
    from shutil import which

    return which(cmd)


def load_tree(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or "tree" not in data:
        raise ValueError("YAML missing top-level 'tree' key")
    tree = data["tree"]
    if not isinstance(tree, dict) or "root" not in tree:
        raise ValueError("'tree' missing 'root' definition")
    return tree


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Visualize behavior tree YAML into Graphviz",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=textwrap.dedent(
            """
            Examples:
              python tools/bt_visualize.py src/rm_bt_decision/config/trees/nav.yaml -o nav.dot
              python tools/bt_visualize.py src/rm_bt_decision/config/trees/nav.yaml -o nav.png --format png
            """
        ),
    )
    parser.add_argument("yaml", help="Path to behavior tree YAML")
    parser.add_argument(
        "-o",
        "--out",
        dest="out",
        default=None,
        help="Output file path (.dot/.png/.svg). Defaults to replacing .yaml extension with .dot",
    )
    parser.add_argument(
        "--format",
        dest="fmt",
        choices=["dot", "png", "svg"],
        default="dot",
        help="Output format (default: dot)",
    )

    args = parser.parse_args(argv)
    in_path = args.yaml
    if not os.path.isfile(in_path):
        print(f"[ERROR] File not found: {in_path}", file=sys.stderr)
        return 2

    tree = load_tree(in_path)
    root = tree["root"]

    dot = DotBuilder()
    root_id = build_graph(dot, root)

    # Add a single hidden root to hold the tree name and point to actual root for better layout
    tree_name = str(tree.get("name", "BehaviorTree"))
    visible_root = dot.new_id()
    dot.add_node(visible_root, f"ROOT\\n{tree_name}", shape="plaintext")
    dot.add_edge(visible_root, root_id)

    dot_text = dot.finish()

    out_path = args.out
    if out_path is None:
        base, _ = os.path.splitext(in_path)
        out_path = base + "." + (args.fmt if args.fmt != "dot" else "dot")

    # Ensure output directory exists
    out_dir = os.path.dirname(os.path.abspath(out_path))
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    render_dot(dot_text, out_path, args.fmt)
    print(f"[OK] Wrote: {out_path}")
    if args.fmt != "dot" and not shutil_which("dot"):
        print("[HINT] Install Graphviz to render images: e.g., sudo apt-get install graphviz", file=sys.stderr)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())

