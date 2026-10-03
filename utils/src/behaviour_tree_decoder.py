class BehaviourTreeDecoder:
    def __init__(self):
        # dict: number (parent_number, name)
        self._nodes: dict[int, tuple[int, str]] = {}    
        self._running_id: int | None = None

    def handle_message(self, text: str) -> None:
        # text is a message from ground, either a tree snapshot or running node id
        if text.startswith("T,"):
            parts = text.split(",")
            if len(parts) != 4:
                raise ValueError(f"Invalid tree message: {text}")
            number = int (parts[1])
            parent = int(parts[2])
            name = parts[3]
            self._nodes[number] = (parent, name)
        elif text.startswith("S,"):
            parts = text.split(",")
            if len(parts) != 2:
                raise ValueError(f"Invalid running node message: {text}")
            self._running_id = int(parts[1])
        pass

    def current_path(self) -> list[str]:
        #start an empty list
        path = []
        #start at self.running_id
        current_id = self._running_id

        while current_id is not None:
            if current_id not in self._nodes:
                raise ValueError(f"Running node id {current_id} not in tree nodes")
            #look up number in self._nodes, get parent and name
            parent, name = self._nodes[current_id]
            path.append(name)
            
            if parent == -1:
                break            #append name to path
            #set current_id to parent
            current_id = parent
        return path
        