from __future__ import annotations


class BehaviourTreeDecoder:
    def __init__(self):
        # dict: number -> (parent_number, name)
        self._nodes: dict[int, tuple[int, str]] = {}
        self._running_id: int | None = None

    def load_table(self, path) -> None:
        # tree table file saved by the airside reporter before takeoff
        with open(path) as file:
            for line in file:
                if line.strip():
                    self.handle_message(line.strip())

    def handle_message(self, text: str) -> None:
        # text is either a tree table line or the running node id
        if text.startswith("T,"):
            parts = text.split(",", 3)
            if len(parts) != 4:
                raise ValueError(f"Invalid tree message: {text}")
            self._nodes[int(parts[1])] = (int(parts[2]), parts[3])
        elif text.startswith("S,"):
            parts = text.split(",")
            if len(parts) != 2:
                raise ValueError(f"Invalid running node message: {text}")
            self._running_id = int(parts[1])

    def _path_ids(self) -> list[int]:
        # node ids from the running node up to the root
        path = []
        current_id = self._running_id
        while current_id is not None:
            if current_id not in self._nodes:
                raise ValueError(f"Running node id {current_id} not in tree nodes")
            path.append(current_id)
            parent, _ = self._nodes[current_id]
            if parent == -1:
                break
            current_id = parent
        return list(reversed(path))  # root first

    def current_path(self) -> list[str]:
        return [self._nodes[number][1] for number in self._path_ids()]

    def render(self) -> str:
        # whole tree, one node per line, running branch marked with *
        running_path = set(self._path_ids())
        children: dict[int, list[int]] = {}
        root = None
        for number, (parent, _) in sorted(self._nodes.items()):
            if parent == -1:
                root = number
            else:
                children.setdefault(parent, []).append(number)

        lines = []

        def visit(number: int, depth: int) -> None:
            mark = "*" if number in running_path else " "
            lines.append(f"{'    ' * depth}{mark} {self._nodes[number][1]}")
            for child in children.get(number, []):
                visit(child, depth + 1)

        if root is not None:
            visit(root, 0)
        return "\n".join(lines)