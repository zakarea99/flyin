from src.model.Connection import Connection
from src.model.Hub import Hub
from math import hypot


class Graph:
    def __init__(self, hubs: list[Hub], connections: list[Connection]):
        self.hubs: dict[str, Hub] = {hub.name: hub for hub in hubs}
        self.adjacency_list: dict[str, list[str]] = {
            hub.name: [] for hub in hubs}
        self.connections_capacities: dict[tuple[str, str], int] = {}
        self._build_graph(connections)

    def _build_graph(self, connections: list[Connection]) -> None:
        for con in connections:
            x = con.from_hub
            y = con.to_hub
            if x not in self.hubs:
                raise ValueError(
                    f"Graph Error: Connection references missing hub '{x}'")
            if y not in self.hubs:
                raise ValueError(
                    f"Graph Error: Connection references missing hub '{y}'")
            self.adjacency_list[x].append(y)
            self.adjacency_list[y].append(x)
            self.connections_capacities[(x, y)] = con.max_link_capacity
            self.connections_capacities[(y, x)] = con.max_link_capacity

    def get_distance(self, hub1_name: str, hub2_name: str) -> float:
        """Calculates the Euclidean distance between two hubs using their x,y coordinates."""
        h1 = self.hubs[hub1_name]
        h2 = self.hubs[hub2_name]
        return hypot(h1.x - h2.x, h1.y - h2.y)