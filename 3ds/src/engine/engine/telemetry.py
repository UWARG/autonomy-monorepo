"""Read-only, versioned snapshot of a py_trees tree (no blackboard mutation)."""
from __future__ import annotations


def snapshot(root, stamp: float) -> dict:
    """Walk pre-order so parents precede children in the dashboard."""
    nodes = []

    def visit(node, parent=None):
        nodes.append({
            "id": str(node.id), "parent": parent,
            "name": node.name, "status": node.status.name,
            "feedback": getattr(node, "feedback_message", ""),
        })
        for child in node.children:
            visit(child, str(node.id))

    visit(root)
    tip = root.tip()
    return {
        "version": 1, "stamp": stamp, "nodes": nodes,
        "tip": str(tip.id) if tip is not None else None,
    }
